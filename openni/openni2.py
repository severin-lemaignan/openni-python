# pylint: disable=W0212,W0603

import sys
import os
import ctypes
import weakref
import atexit
import platform
from openni import _openni2 as c_api
from openni.utils import (inherit_properties, HandleObject, _py_to_ctype_obj, ClosedHandle, InitializationError,
                          OpenNIError)


arch = int(platform.architecture()[0].lower().replace("bit", ""))

_default_dll_directories = []
if arch == 32:
    if "OPENNI2_REDIST" in os.environ:
        _default_dll_directories.append(os.environ["OPENNI2_REDIST"])
elif arch == 64:
    if "OPENNI2_REDIST64" in os.environ:
        _default_dll_directories.append(os.environ["OPENNI2_REDIST64"])
    elif "OPENNI2_REDIST" in os.environ:
        _default_dll_directories.append(os.environ["OPENNI2_REDIST"])

_default_dll_directories.append(".")

if sys.platform == "win32":
    _dll_name = "OpenNI2.dll"
elif sys.platform == "darwin":
    _dll_name = "libOpenNI2.dylib"
else:  # GNU/Linux, *BSD, etc
    _default_dll_directories += [
        "/lib",
        "/usr/lib",
        "/usr/local/lib",
    ]
    if arch == 64:
        _default_dll_directories.append("/lib64")
    _dll_name = "libOpenNI2.so"


SENSOR_IR = c_api.OniSensorType.ONI_SENSOR_IR
SENSOR_COLOR = c_api.OniSensorType.ONI_SENSOR_COLOR
SENSOR_DEPTH = c_api.OniSensorType.ONI_SENSOR_DEPTH

PIXEL_FORMAT_DEPTH_1_MM = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM
PIXEL_FORMAT_DEPTH_100_UM = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM
PIXEL_FORMAT_SHIFT_9_2 = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_SHIFT_9_2
PIXEL_FORMAT_SHIFT_9_3 = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_SHIFT_9_3
PIXEL_FORMAT_RGB888 = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888
PIXEL_FORMAT_YUV422 = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_YUV422
PIXEL_FORMAT_GRAY8 = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_GRAY8
PIXEL_FORMAT_GRAY16 = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_GRAY16
PIXEL_FORMAT_JPEG = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_JPEG
PIXEL_FORMAT_YUYV = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_YUYV

IMAGE_REGISTRATION_DEPTH_TO_COLOR = c_api.OniImageRegistrationMode.ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR
IMAGE_REGISTRATION_OFF = c_api.OniImageRegistrationMode.ONI_IMAGE_REGISTRATION_OFF


_openni2_initialized = False
loaded_dll_directory = None


def initialize(dll_directories=_default_dll_directories):
    global _openni2_initialized
    global loaded_dll_directory
    if _openni2_initialized:
        return
    if isinstance(dll_directories, str):
        dll_directories = [dll_directories]
    if loaded_dll_directory:
        c_api.oniInitialize(c_api.ONI_API_VERSION)
        _openni2_initialized = True
        return

    found = False
    prev = os.getcwd()
    exceptions = []
    dll_directories = [os.path.normpath(os.path.abspath(d)) for d in dll_directories]

    for dlldir in dll_directories:
        if not os.path.isdir(dlldir):
            exceptions.append((dlldir, "Directory does not exist"))
            continue
        fullpath = os.path.join(dlldir, _dll_name)
        if not os.path.isfile(fullpath):
            exceptions.append((fullpath, "file does not exist"))
            continue
        try:
            os.chdir(dlldir)
            c_api.load_dll(fullpath)
            c_api.oniInitialize(c_api.ONI_API_VERSION)
        except Exception as ex:
            exceptions.append((fullpath, ex))
        else:
            found = True
            loaded_dll_directory = dlldir
            break

    os.chdir(prev)
    if not found:
        raise InitializationError("OpenNI2 could not be loaded:\n    %s" %
                                  ("\n    ".join("%s: %s" % (dir, ex) for dir, ex in exceptions)),)

    _openni2_initialized = True


def is_initialized():
    return _openni2_initialized


_registered_devices = weakref.WeakSet()
_registered_video_frames = weakref.WeakSet()
_registered_video_streams = weakref.WeakSet()
_registered_recorders = weakref.WeakSet()
_registered_device_listeners = weakref.WeakSet()


def unload():
    global _openni2_initialized
    if not _openni2_initialized:
        return
    for coll in [_registered_video_frames, _registered_recorders, _registered_video_streams, _registered_device_listeners,
                 _registered_devices]:
        for hndl in coll:
            hndl.close()
        coll.clear()

    _openni2_initialized = False
    c_api.oniShutdown()


atexit.register(unload)


def get_version():
    return c_api.oniGetVersion()


def wait_for_any_stream(streams, timeout=None):
    if timeout is None:
        timeout = c_api.ONI_TIMEOUT_FOREVER
    else:
        timeout = int(timeout * 1000)  # in msec
    ready_stream_index = ctypes.c_int(-1)
    arr = (c_api.OniStreamHandle * len(streams))()
    for i, s in enumerate(streams):
        arr[i] = s._handle
    try:
        c_api.oniWaitForAnyStream(arr, len(streams), ctypes.byref(ready_stream_index), timeout)
    except OpenNIError as ex:
        if ex.code == c_api.OniStatus.ONI_STATUS_TIME_OUT:
            # timed out
            return None
        else:
            raise
    if ready_stream_index.value >= 0:
        return streams[ready_stream_index.value]
    else:
        return None


VideoMode = c_api.OniVideoMode
DeviceInfo = c_api.OniDeviceInfo


class SensorInfo(object):
    def __init__(self, info):
        self.sensorType = info.sensorType
        self.videoModes = [info.pSupportedVideoModes[i] for i in range(info.numSupportedVideoModes)]

    @classmethod
    def from_stream_handle(cls, handle):
        pinfo = c_api.oniStreamGetSensorInfo(handle)
        if pinfo == 0:
            return None
        return cls(pinfo[0])

    @classmethod
    def from_device_handle(cls, handle, sensor_type):
        pinfo = c_api.oniDeviceGetSensorInfo(handle, sensor_type)
        if not pinfo:
            return None
        return cls(pinfo[0])


class PlaybackSupport(object):
    __slots__ = ["device"]

    def __init__(self, device):
        self.device = weakref.proxy(device)

    def get_speed(self):
        return self.device.get_property(c_api.ONI_DEVICE_PROPERTY_PLAYBACK_SPEED, ctypes.c_float)

    def set_speed(self, speed):
        return self.device.set_property(c_api.ONI_DEVICE_PROPERTY_PLAYBACK_SPEED, speed)
    speed = property(get_speed, set_speed)

    def get_repeat_enabled(self):
        return bool(self.device.get_property(c_api.ONI_DEVICE_PROPERTY_PLAYBACK_REPEAT_ENABLED, c_api.OniBool))

    def set_repeat_enabled(self, enable):
        self.device.set_property(c_api.ONI_DEVICE_PROPERTY_PLAYBACK_REPEAT_ENABLED, enable)
    repeat = property(get_repeat_enabled, set_repeat_enabled)

    def seek(self, stream, frame_index):
        seek = c_api.OniSeek(frameIndex=frame_index, stream=stream._handle)
        self.device.invoke(c_api.ONI_DEVICE_COMMAND_SEEK, seek)

    def get_number_of_frames(self, stream):
        return stream.get_number_of_frames()


class Device(HandleObject):
    def __init__(self, uri, mode=None):
        self._orig_uri = uri
        self.uri = uri
        self._mode = mode
        HandleObject.__init__(self, None)
        self._reopen()
        _registered_devices.add(self)

    def _reopen(self):
        self.close()
        self._handle = c_api.OniDeviceHandle()
        assert not bool(self._handle)
        if self._mode:
            c_api.oniDeviceOpenEx(self._orig_uri, self._mode, ctypes.byref(self._handle))
        else:
            c_api.oniDeviceOpen(self._orig_uri, ctypes.byref(self._handle))
        assert bool(self._handle), "Handle is NULL after open"
        if self.is_file():
            self.playback = PlaybackSupport(self)
        else:
            self.playback = None
        self._sensor_infos = {}
        self._devinfo = None
        self.get_device_info()

    @classmethod
    def enumerate_uris(cls):
        pdevs = ctypes.POINTER(c_api.OniDeviceInfo)()
        count = ctypes.c_int()
        c_api.oniGetDeviceList(ctypes.byref(pdevs), ctypes.byref(count))
        uris = [pdevs[i].uri for i in range(count.value)]
        c_api.oniReleaseDeviceList(pdevs)
        return uris

    @classmethod
    def open_all(cls):
        return [cls(uri) for uri in cls.enumerate_uris()]

    @classmethod
    def open_any(cls):
        return cls(None)

    @classmethod
    def open_file(cls, filename):
        return cls(filename)

    def _close(self):
        if is_initialized():
            c_api.oniDeviceClose(self._handle)
        self.playback = None

    def get_device_info(self):
        if self._devinfo is None:
            self._devinfo = c_api.OniDeviceInfo()
            c_api.oniDeviceGetInfo(self._handle, ctypes.byref(self._devinfo))
            self.uri = self._devinfo.uri
        return self._devinfo
    device_info = property(get_device_info)

    def get_sensor_info(self, sensor_type):
        if sensor_type in self._sensor_infos:
            return self._sensor_infos[sensor_type]

        info = SensorInfo.from_device_handle(self._handle, sensor_type)
        self._sensor_infos[sensor_type] = info
        return info

    def has_sensor(self, sensor_type):
        return self.get_sensor_info(sensor_type) is not None

    def create_stream(self, sensor_type):
        return VideoStream(self, sensor_type)

    def create_depth_stream(self):
        if not self.has_sensor(SENSOR_DEPTH):
            return None
        return VideoStream(self, SENSOR_DEPTH)

    def create_color_stream(self):
        if not self.has_sensor(SENSOR_COLOR):
            return None
        return VideoStream(self, SENSOR_COLOR)

    def create_ir_stream(self):
        if not self.has_sensor(SENSOR_IR):
            return None
        return VideoStream(self, SENSOR_IR)

    def get_property(self, property_id, rettype):
        ret = rettype()
        size = ctypes.c_int(ctypes.sizeof(ret))
        c_api.oniDeviceGetProperty(self._handle, property_id, ctypes.byref(ret), ctypes.byref(size))
        return ret

    def get_int_property(self, property_id):
        return self.get_property(property_id, ctypes.c_int).value

    def set_property(self, property_id, obj, size=None):
        obj, size = _py_to_ctype_obj(obj)
        if size is None:
            size = ctypes.sizeof(obj)
        c_api.oniDeviceSetProperty(self._handle, property_id, ctypes.byref(obj), size)

    def is_property_supported(self, property_id):
        return bool(c_api.oniDeviceIsPropertySupported(self._handle, property_id))

    def invoke(self, command_id, data, size=None):
        data, size = _py_to_ctype_obj(data)
        if size is None:
            size = ctypes.sizeof(data)
        c_api.oniDeviceInvoke(self._handle, command_id, ctypes.byref(data), size)

    def is_command_supported(self, command_id):
        return bool(c_api.oniDeviceIsCommandSupported(self._handle, command_id))

    def is_image_registration_mode_supported(self, mode):
        return bool(c_api.oniDeviceIsImageRegistrationModeSupported(self._handle, mode))

    def get_image_registration_mode(self):
        return self.get_property(c_api.ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION, c_api.OniImageRegistrationMode)

    def set_image_registration_mode(self, mode):
        self.set_property(c_api.ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION, mode)

    def is_file(self):
        return (self.is_property_supported(c_api.ONI_DEVICE_PROPERTY_PLAYBACK_SPEED) and
                self.is_property_supported(c_api.ONI_DEVICE_PROPERTY_PLAYBACK_REPEAT_ENABLED) and
                self.is_command_supported(c_api.ONI_DEVICE_COMMAND_SEEK))

    def get_depth_color_sync_enabled(self):
        return bool(c_api.oniDeviceGetDepthColorSyncEnabled(self._handle))

    def set_depth_color_sync_enabled(self, enable):
        if enable:
            c_api.oniDeviceEnableDepthColorSync(self._handle)
        else:
            c_api.oniDeviceDisableDepthColorSync(self._handle)
    depth_color_sync = property(get_depth_color_sync_enabled, set_depth_color_sync_enabled)


@inherit_properties(c_api.OniFrame, "_frame")
class VideoFrame(HandleObject):
    def __init__(self, pframe):
        self._frame = pframe[0]
        HandleObject.__init__(self, pframe)
        _registered_video_frames.add(self)

    def _close(self):
        if is_initialized():
            c_api.oniFrameRelease(self._handle)
        self._frame = ClosedHandle

    def get_buffer_as(self, ctype):
        return (ctype * int(self.dataSize / ctypes.sizeof(ctype))).from_address(self.data)

    def get_buffer_as_uint8(self):
        return self.get_buffer_as(ctypes.c_uint8)

    def get_buffer_as_uint16(self):
        return self.get_buffer_as(ctypes.c_uint16)

    def get_buffer_as_triplet(self):
        return self.get_buffer_as(ctypes.c_uint8 * 3)


class CameraSettings(object):
    __slots__ = ["stream"]

    def __init__(self, stream):
        self.stream = weakref.proxy(stream)

    def get_auto_exposure(self):
        return bool(self.stream.get_property(c_api.ONI_STREAM_PROPERTY_AUTO_EXPOSURE, c_api.OniBool))

    def set_auto_exposure(self, enabled):
        self.stream.set_property(c_api.ONI_STREAM_PROPERTY_AUTO_EXPOSURE, enabled)
    auto_exposure = property(get_auto_exposure, set_auto_exposure)

    def get_auto_white_balance(self):
        return bool(self.stream.get_property(c_api.ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE, c_api.OniBool))

    def set_auto_white_balance(self, enabled):
        return self.stream.set_property(c_api.ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE, enabled)
    auto_white_balance = property(get_auto_white_balance, set_auto_white_balance)

    def get_gain(self):
        try:
            return self.stream.get_property(c_api.ONI_STREAM_PROPERTY_GAIN, ctypes.c_int).value
        except OpenNIError:
            return 100

    def set_gain(self, gain):
        return self.stream.set_property(c_api.ONI_STREAM_PROPERTY_GAIN, gain)
    gain = property(get_gain, set_gain)

    def get_exposure(self):
        try:
            return self.stream.get_property(c_api.ONI_STREAM_PROPERTY_EXPOSURE, ctypes.c_int).value
        except OpenNIError:
            return 0

    def set_exposure(self, exposure):
        return self.stream.set_property(c_api.ONI_STREAM_PROPERTY_EXPOSURE, exposure)
    exposure = property(get_exposure, set_exposure)


class VideoStream(HandleObject):
    def __init__(self, device, sensor_type):
        self.device = device
        self.sensor_type = sensor_type
        self._callbacks = {}
        handle = c_api.OniStreamHandle()
        c_api.oniDeviceCreateStream(self.device._handle, sensor_type, ctypes.byref(handle))
        HandleObject.__init__(self, handle)
        _registered_video_streams.add(self)
        if (self.is_property_supported(c_api.ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE) and
                self.is_property_supported(c_api.ONI_STREAM_PROPERTY_AUTO_EXPOSURE)):
            self.camera = CameraSettings(self)
        else:
            self.camera = None

    def _close(self):
        if is_initialized():
            self.unregister_all_new_frame_listeners()
            self.stop()
            c_api.oniStreamDestroy(self._handle)
        self.camera = None

    def get_sensor_info(self):
        return SensorInfo.from_stream_handle(self._handle)

    def get_recoder(self, filename, allow_lossy_compression=False):
        rec = Recorder(filename)
        rec.attach(self, allow_lossy_compression)
        return rec

    def start(self):
        c_api.oniStreamStart(self._handle)

    def stop(self):
        c_api.oniStreamStop(self._handle)

    def read_frame(self):
        pframe = ctypes.POINTER(c_api.OniFrame)()
        c_api.oniStreamReadFrame(self._handle, ctypes.byref(pframe))
        return VideoFrame(pframe)

    def register_new_frame_listener(self, callback):
        """callback(stream : VideoStream) -> None"""
        if callback in self._callbacks:
            raise ValueError("Callback %r already registered" % (callback,))

        def adapter(handle, cookie):
            callback(self)

        cb_handle = c_api.OniCallbackHandle()
        cbobj = c_api.OniNewFrameCallback(adapter)
        self._callbacks[callback] = (cb_handle, adapter, cbobj)
        c_api.oniStreamRegisterNewFrameCallback(self._handle, cbobj, None, ctypes.byref(cb_handle))

    def unregister_new_frame_listener(self, callback):
        if callback not in self._callbacks:
            return
        cb_handle, _, _ = self._callbacks.pop(callback)
        c_api.oniStreamUnregisterNewFrameCallback(self._handle, cb_handle)

    def unregister_all_new_frame_listeners(self):
        for cb_handle, _, _ in self._callbacks.values():
            c_api.oniStreamUnregisterNewFrameCallback(self._handle, cb_handle)
        self._callbacks.clear()

    def get_property(self, property_id, rettype):
        ret = rettype()
        size = ctypes.c_int(ctypes.sizeof(ret))
        c_api.oniStreamGetProperty(self._handle, property_id, ctypes.byref(ret), ctypes.byref(size))
        return ret

    def get_int_property(self, property_id):
        return self.get_property(property_id, ctypes.c_int).value

    def set_property(self, property_id, obj, size=None):
        obj, size = _py_to_ctype_obj(obj)
        if size is None:
            size = ctypes.sizeof(obj)
        c_api.oniStreamSetProperty(self._handle, property_id, ctypes.byref(obj), size)

    def is_property_supported(self, property_id):
        return bool(c_api.oniStreamIsPropertySupported(self._handle, property_id))

    def invoke(self, command_id, data, size=None):
        data, size = _py_to_ctype_obj(data)
        if size is None:
            size = ctypes.sizeof(data)
        c_api.oniStreamInvoke(self._handle, command_id, data, size)

    def is_command_supported(self, command_id):
        return bool(c_api.oniStreamIsCommandSupported(self._handle, command_id))

    def get_video_mode(self):
        return self.get_property(c_api.ONI_STREAM_PROPERTY_VIDEO_MODE, c_api.OniVideoMode)

    def set_video_mode(self, video_mode):
        self.set_property(c_api.ONI_STREAM_PROPERTY_VIDEO_MODE, video_mode)
    video_mode = property(get_video_mode, set_video_mode)

    def configure_mode(self, width, height, fps, pixel_format):
        """shortcut for set_video_mode"""
        mode = VideoMode(resolutionX=width, resolutionY=height, fps=fps, pixelFormat=pixel_format)
        self.set_video_mode(mode)

    def get_max_pixel_value(self):
        return self.get_int_property(c_api.ONI_STREAM_PROPERTY_MAX_VALUE)

    def get_min_pixel_value(self):
        return self.get_int_property(c_api.ONI_STREAM_PROPERTY_MIN_VALUE)

    def is_cropping_supported(self):
        return self.is_property_supported(c_api.ONI_STREAM_PROPERTY_CROPPING)

    def get_cropping(self):
        return self.get_property(c_api.ONI_STREAM_PROPERTY_CROPPING, c_api.OniCropping)

    def set_cropping(self, originX, originY, width, height):
        cropping = c_api.OniCropping(enabled=True, originX=originX, originY=originY, width=width, height=height)
        self.set_property(c_api.ONI_STREAM_PROPERTY_CROPPING, cropping)
    cropping = property(get_property, set_property)

    def reset_cropping(self):
        self.set_property(c_api.ONI_STREAM_PROPERTY_CROPPING, c_api.OniCropping(enabled=False))

    def get_mirroring_enabled(self):
        return bool(self.get_property(c_api.ONI_STREAM_PROPERTY_MIRRORING, c_api.OniBool))

    def set_mirroring_enabled(self, enabled):
        self.set_property(c_api.ONI_STREAM_PROPERTY_MIRRORING, enabled)
    mirroring_enabled = property(get_mirroring_enabled, set_mirroring_enabled)

    def get_horizontal_fov(self):
        return self.get_property(c_api.ONI_STREAM_PROPERTY_HORIZONTAL_FOV, ctypes.c_float).value

    def get_vertical_fov(self):
        return self.get_property(c_api.ONI_STREAM_PROPERTY_VERTICAL_FOV, ctypes.c_float).value

    def get_number_of_frames(self):
        return self.get_int_property(c_api.ONI_STREAM_PROPERTY_NUMBER_OF_FRAMES)

    def set_frame_buffers_allocator(self, allocator=None):
        if not allocator:
            c_api.oniStreamSetFrameBuffersAllocator(self._handle, None, None, None)
        else:
            return c_api.oniStreamSetFrameBuffersAllocator(self._handle,
                                                           allocator._allocate_callback, allocator._free_callback, None)


class FrameAllocator(object):
    def __init__(self):
        # keep reference to the methods (they are passed as callbacks)
        self._alloc_callback = c_api.OniFrameAllocBufferCallback(self._allocate_frame_buffer_callback)
        self._free_callback = c_api.OniFrameFreeBufferCallback(self._free_frame_buffer_callback)

    def allocate_frame_buffer(self, size):
        raise NotImplementedError()

    def free_frame_buffer(self, pdata):
        raise NotImplementedError()

    def _allocate_frame_buffer_callback(self, size, _):
        return self.allocate_frame_buffer(size)

    def _free_frame_buffer_callback(self, pdata, _):
        return self.free_frame_buffer(pdata)


class Recorder(HandleObject):
    def __init__(self, filename):
        self.filename = filename
        handle = c_api.OniRecorderHandle()
        c_api.oniCreateRecorder(filename, ctypes.byref(handle))
        HandleObject.__init__(self, handle)
        _registered_recorders.add(self)

    def _close(self):
        if is_initialized():
            c_api.oniRecorderDestroy(ctypes.byref(self._handle))

    def attach(self, stream, allow_lossy_compression=False):
        c_api.oniRecorderAttachStream(self._handle, stream._handle, allow_lossy_compression)

    def start(self):
        c_api.oniRecorderStart(self._handle)

    def stop(self):
        c_api.oniRecorderStop(self._handle)


def convert_world_to_depth(depthStream, worldX, worldY, worldZ):
    """const VideoStream& depthStream, float worldX, float worldY, float worldZ"""
    out_depthX = ctypes.c_float()
    out_depthY = ctypes.c_float()
    out_depthZ = ctypes.c_float()
    c_api.oniCoordinateConverterWorldToDepth(depthStream._handle, worldX, worldY, worldZ,
                                             ctypes.byref(out_depthX), ctypes.byref(out_depthY), ctypes.byref(out_depthZ))
    return out_depthX.value, out_depthY.value, out_depthZ.value


def convert_depth_to_world(depthStream, depthX, depthY, depthZ):
    """const VideoStream& depthStream, float depthX, float depthY, float depthZ, float* pWorldX, float* pWorldY, float* pWorldZ"""
    out_depthX = ctypes.c_float()
    out_depthY = ctypes.c_float()
    out_depthZ = ctypes.c_float()
    c_api.oniCoordinateConverterDepthToWorld(depthStream._handle, depthX, depthY, depthZ,
                                             ctypes.byref(out_depthX), ctypes.byref(out_depthY), ctypes.byref(out_depthZ))
    return out_depthX.value, out_depthY.value, out_depthZ.value


def convert_depth_to_color(depthStream, colorStream, depthX, depthY, depthZ):
    """const VideoStream& depthStream, const VideoStream& colorStream, int depthX, int depthY, DepthPixel depthZ, int* pColorX, int* pColorY"""
    colorX = ctypes.c_int()
    colorY = ctypes.c_int()
    c_api.oniCoordinateConverterDepthToColor(depthStream._handle, colorStream._handle, depthX, depthY, depthZ,
                                             ctypes.byref(colorX), ctypes.byref(colorY))
    return colorX.value, colorY.value


def get_bytes_per_pixel(format):
    c_api.oniFormatBytesPerPixel(format)


class DeviceListener(HandleObject):
    def __init__(self):
        handle = c_api.OniCallbackHandle()
        self._callbacks = c_api.OniDeviceCallbacks(
            deviceConnected=c_api.OniDeviceInfoCallback(self._on_connected),
            deviceDisconnected=c_api.OniDeviceInfoCallback(self._on_disconnected),
            deviceStateChanged=c_api.OniDeviceStateCallback(self._on_state_changed),
        )
        c_api.oniRegisterDeviceCallbacks(self._callbacks, None, ctypes.byref(handle))
        HandleObject.__init__(self, handle)
        _registered_device_listeners.add(self)
        self._connected_uris = set()
        self._disconnected_uris = set()

    def _close(self):
        if is_initialized():
            c_api.oniUnregisterDeviceCallbacks(self._handle)

    def unregister(self):
        self.close()

    def _on_connected(self, pdevinfo, _):
        devinfo = pdevinfo[0]
        self._disconnected_uris.discard(devinfo.uri)
        if devinfo.uri in self._connected_uris:
            return
        self._connected_uris.add(devinfo.uri)
        self.on_connected(devinfo)

    def _on_disconnected(self, pdevinfo, _):
        devinfo = pdevinfo[0]
        self._connected_uris.discard(devinfo.uri)
        if devinfo.uri in self._disconnected_uris:
            return
        self._disconnected_uris.add(devinfo.uri)
        self.on_disconnected(devinfo)

    def _on_state_changed(self, pdevinfo, state, _):
        self.on_state_changed(pdevinfo[0], state)

    def on_connected(self, devinfo):
        """Implement me"""
        pass

    def on_disconnected(self, devinfo):
        """Implement me"""
        pass

    def on_state_changed(self, devinfo, state):
        """Implement me"""
        pass


def get_log_filename():
    buf = ctypes.create_string_buffer(1024)
    try:
        c_api.oniGetLogFileName(buf, ctypes.sizeof(buf))
    except OpenNIError:
        # not logging to file
        return None
    else:
        return buf.value


def configure_logging(directory=None, severity=None, console=None):
    """
    directory: directory in which log files will be stored
    severity: 0 - Verbose; 1 - Info; 2 - Warning; 3 - Error. Default - None
    console: whether to print to the console (boolean)
    """
    if directory is not None:
        c_api.oniSetLogOutputFolder(directory)
        c_api.oniSetLogFileOutput(True)
    else:
        c_api.oniSetLogFileOutput(False)

    if severity is not None:
        c_api.oniSetLogMinSeverity(severity)

    if console is not None:
        c_api.oniSetLogConsoleOutput(bool(console))
