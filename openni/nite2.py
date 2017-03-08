# pylint: disable=W0212,W0603

import sys
import os
import weakref
import atexit
import ctypes
import platform
from openni import _nite2 as c_api
from openni import openni2
from openni.utils import inherit_properties, ClosedHandle, HandleObject, InitializationError

from openni._nite2 import NiteJointType as JointType
from openni._nite2 import NiteSkeletonState as SkeletonState
from openni._nite2 import NiteUserState as UserState
from openni._nite2 import NitePoseType as PoseType
from openni._nite2 import NiteGestureType as GestureType

arch = int(platform.architecture()[0].lower().replace("bit", ""))

_default_dll_directories = []
if arch == 32:
    if "NITE2_REDIST" in os.environ:
        _default_dll_directories.append(os.environ["NITE2_REDIST"])
elif arch == 64:
    if "NITE2_REDIST64" in os.environ:
        _default_dll_directories.append(os.environ["NITE2_REDIST64"])
    elif "NITE2_REDIST" in os.environ:
        _default_dll_directories.append(os.environ["NITE2_REDIST"])

_default_dll_directories.append(".")

if sys.platform == "win32":
    _dll_name = "NiTE2.dll"
elif sys.platform == "darwin":
    _dll_name = "libNiTE2.dylib"
else:
    _dll_name = "libNiTE2.so"


_nite2_initialized = False
loaded_dll_directory = None


def initialize(dll_directories=_default_dll_directories):
    global _nite2_initialized
    global loaded_dll_directory
    if _nite2_initialized:
        return
    if isinstance(dll_directories, str):
        dll_directories = [dll_directories]

    if not openni2.is_initialized():
        openni2.initialize()

    if loaded_dll_directory:
        c_api.niteInitialize()
        _nite2_initialized = True
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
            c_api.niteInitialize()
        except Exception as ex:
            exceptions.append((fullpath, ex))
        else:
            found = True
            loaded_dll_directory = dlldir
            break

    os.chdir(prev)
    if not found:
        raise InitializationError("NiTE2 could not be loaded:\n    %s" %
                                  ("\n    ".join("%s: %s" % (dir, ex) for dir, ex in exceptions)),)

    _nite2_initialized = True


def is_initialized():
    return _nite2_initialized


_registered_user_trackers = weakref.WeakSet()
_registered_user_tracker_frames = weakref.WeakSet()
_registered_hand_trackers = weakref.WeakSet()
_registered_hand_tracker_frames = weakref.WeakSet()
_registered_user_tracker_listeners = weakref.WeakSet()
_registered_hand_tracker_listeners = weakref.WeakSet()


def unload():
    global _nite2_initialized
    if not _nite2_initialized:
        return
    for coll in [_registered_user_tracker_frames, _registered_hand_tracker_frames, _registered_hand_trackers,
                 _registered_user_trackers, _registered_user_tracker_listeners, _registered_hand_tracker_listeners]:
        for hndl in coll:
            hndl.close()
        coll.clear()

    _nite2_initialized = False
    c_api.niteShutdown()


atexit.register(unload)


def get_version():
    return c_api.niteGetVersion()


Point3f = c_api.NitePoint3f
Plane = c_api.NitePlane
Quaternion = c_api.NiteQuaternion
BoundingBox = c_api.NiteBoundingBox
UserId = c_api.NiteUserId
HandId = c_api.NiteHandId
UserMap = c_api.NiteUserMap
SkeletonJoint = c_api.NiteSkeletonJoint


@inherit_properties(c_api.NitePoseData, "_posedata")
class PoseData(object):
    __slots__ = ["_posedata"]

    def __init__(self, posedata):
        self._posedata = posedata

    def is_held(self):
        return self.state == c_api.NitePoseState.NITE_POSE_STATE_IN_POSE

    def is_entered(self):
        return self.state == c_api.NitePoseState.NITE_POSE_STATE_ENTER

    def is_exited(self):
        return self.state == c_api.NitePoseState.NITE_POSE_STATE_EXIT


@inherit_properties(c_api.NiteSkeleton, "_skeleton")
class Skeleton(object):
    __slots__ = ["_skeleton"]

    def __init__(self, skeleton):
        self._skeleton = skeleton

    def get_joint(self, jointtype):
        return self.joints[jointtype]


@inherit_properties(c_api.NiteUserData, "_userdata")
class UserData(object):
    __slots__ = ["_userdata"]

    def __init__(self, userdata):
        self._userdata = userdata

    def is_new(self):
        return self.state == c_api.NiteUserState.NITE_USER_STATE_NEW

    def is_visible(self):
        return self.state == c_api.NiteUserState.NITE_USER_STATE_VISIBLE

    def is_lost(self):
        return self.state == c_api.NiteUserState.NITE_USER_STATE_LOST

    def get_pose(self, posetype):
        return PoseData(self.poses[posetype])


@inherit_properties(c_api.NiteUserTrackerFrame, "_frame")
class UserTrackerFrame(HandleObject):
    __slots__ = ["_frame", "_user_tracker_handle", "_depth_frame", "users", "users_by_id", "__weakref__"]

    def __init__(self, pframe, user_tracker_handle):
        self._frame = pframe[0]
        self._user_tracker_handle = user_tracker_handle
        self._depth_frame = None
        c_api.niteUserTrackerFrameAddRef(user_tracker_handle, pframe)
        HandleObject.__init__(self, pframe)
        self.users = []
        self.users_by_id = {}
        for i in range(self.userCount):
            u = UserData(self.pUser[i])
            self.users.append(u)
            self.users_by_id[u.id] = u
        _registered_user_tracker_frames.add(self)

    def _close(self):
        if is_initialized():
            c_api.niteUserTrackerFrameRelease(self._user_tracker_handle, self._handle)
        self._frame = ClosedHandle
        self._user_tracker_handle = ClosedHandle
        del self.users[:]

    def get_depth_frame(self):
        if self._depth_frame is None:
            self._depth_frame = openni2.VideoFrame(self.pDepthFrame)
        return self._depth_frame


class _NiteDevStruct(ctypes.Structure):
    """
    PlaybackControl* m_pPlaybackControl;

    OniDeviceHandle m_device;
    DeviceInfo m_deviceInfo;
    SensorInfo m_aSensorInfo[ONI_MAX_SENSORS];

    bool m_isOwner;
    """

    _fields_ = [
        ("pPlaybackControl", ctypes.c_void_p),
        ("device", openni2.c_api.OniDeviceHandle),

        ("device_info", openni2.c_api.OniDeviceInfo),
        ("sensor_info", openni2.c_api.OniSensorInfo * 10),

        ("is_owner", ctypes.c_bool)
    ]


class UserTracker(HandleObject):
    def __init__(self, device):
        handle = c_api.NiteUserTrackerHandle()
        if not device:
            c_api.niteInitializeUserTracker(ctypes.byref(handle))
        else:
            self._devstruct = _NiteDevStruct()
            self._devstruct.pPlaybackControl = None
            self._devstruct.device = device._handle
            self._devstruct.device_info = device.get_device_info()
            # self._devstruct.sensor_info = device.get_sensor_infos(sensor_type)
            self._devstruct.is_owner = True
            c_api.niteInitializeUserTrackerByDevice(ctypes.byref(self._devstruct), ctypes.byref(handle))

        HandleObject.__init__(self, handle)
        _registered_user_trackers.add(self)

    @classmethod
    def open_any(cls):
        return UserTracker(None)

    def _close(self):
        if is_initialized():
            c_api.niteShutdownUserTracker(self._handle)

    def read_frame(self):
        pnf = ctypes.POINTER(c_api.NiteUserTrackerFrame)()
        c_api.niteReadUserTrackerFrame(self._handle, ctypes.byref(pnf))
        return UserTrackerFrame(pnf, self._handle)

    def set_skeleton_smoothing_factor(self, factor):
        return c_api.niteSetSkeletonSmoothing(self._handle, factor)

    def get_skeleton_smoothing_factor(self):
        factor = ctypes.c_float()
        c_api.niteGetSkeletonSmoothing(self._handle, ctypes.byref(factor))
        return factor.value
    skeleton_smoothing_factor = property(get_skeleton_smoothing_factor, set_skeleton_smoothing_factor)

    def start_skeleton_tracking(self, userid):
        c_api.niteStartSkeletonTracking(self._handle, userid)

    def stop_skeleton_tracking(self, userid):
        c_api.niteStopSkeletonTracking(self._handle, userid)

    def is_tracking(self, userid):
        c_api.niteIsSkeletonTracking(self._handle, userid)

    def start_pose_detection(self, userid, posetype):
        c_api.niteStartPoseDetection(self._handle, userid, posetype)

    def stop_pose_detection(self, userid, posetype):
        c_api.niteStopPoseDetection(self._handle, userid, posetype)

    def stop_all_pose_detection(self, userid):
        c_api.niteStopAllPoseDetection(self._handle, userid)

    def convert_joint_coordinates_to_depth(self,  x, y, z):
        outX = ctypes.c_float()
        outY = ctypes.c_float()
        c_api.niteConvertJointCoordinatesToDepth(self._handle, x, y, z, ctypes.byref(outX), ctypes.byref(outY))
        return (outX.value, outY.value)

    def convert_depth_coordinates_to_joint(self, x, y, z):
        outX = ctypes.c_float()
        outY = ctypes.c_float()
        c_api.niteConvertDepthCoordinatesToJoint(self._handle, x, y, z, ctypes.byref(outX), ctypes.byref(outY))
        return (outX.value, outY.value)


@inherit_properties(c_api.NiteGestureData, "_gesture")
class GestureData(object):
    def __init__(self, gesture):
        self._gesture = gesture

    def is_complete(self):
        return self.state == c_api.NiteGestureState.NITE_GESTURE_STATE_COMPLETED

    def is_in_progress(self):
        return self.state == c_api.NiteGestureState.NITE_GESTURE_STATE_IN_PROGRESS


@inherit_properties(c_api.NiteHandData, "_handdata")
class HandData(object):
    def __init__(self, handdata):
        self._handdata = handdata

    def is_new(self):
        return self.state == c_api.NiteHandState.NITE_HAND_STATE_NEW

    def is_lost(self):
        return self.state == c_api.NiteHandState.NITE_HAND_STATE_LOST

    def is_tracking(self):
        return self.state == c_api.NiteHandState.NITE_HAND_STATE_TRACKED

    def is_touching_fov(self):
        return self.state == c_api.NiteHandState.NITE_HAND_STATE_TOUCHING_FOV


@inherit_properties(c_api.NiteHandTrackerFrame, "_frame")
class HandTrackerFrame(HandleObject):
    def __init__(self, hand_tracker_handle, pframe):
        self._hand_tracker_handle = hand_tracker_handle
        self._frame = pframe[0]
        c_api.niteHandTrackerFrameAddRef(hand_tracker_handle, pframe)
        HandleObject.__init__(self, pframe)
        self._depth_frame = None
        self._hands = None
        self._gestures = None
        _registered_hand_tracker_frames.add(self)

    def _close(self):
        if is_initialized():
            c_api.niteHandTrackerFrameRelease(self._hand_tracker_handle, self._handle)

    @property
    def depth_frame(self):
        if self._depth_frame is None:
            self._depth_frame = openni2.VideoFrame(self._frame.pDepthFrame)
        return self._depth_frame

    @property
    def hands(self):
        if self._hands is None:
            self._hands = [self._frame.pHands[i] for i in range(self._frame.handCount)]
        return self._hands

    @property
    def gestures(self):
        if self._gestures is None:
            self._gestures = [self._frame.pGestures[i] for i in range(self._frame.gestureCount)]
        return self._gestures


class HandTracker(HandleObject):
    def __init__(self, device):
        self.device = device
        handle = c_api.NiteHandTrackerHandle()
        if not device:
            c_api.niteInitializeHandTracker(ctypes.byref(handle))
        else:
            self._devstruct = _NiteDevStruct()
            self._devstruct.device = device._handle
            c_api.niteInitializeHandTrackerByDevice(ctypes.byref(self._devstruct), ctypes.byref(handle))
        HandleObject.__init__(self, handle)
        _registered_hand_trackers.add(self)

    @classmethod
    def open_any(cls):
        return cls(None)

    def _close(self):
        if is_initialized():
            c_api.niteShutdownHandTracker(self._handle)

    def read_frame(self):
        pfrm = ctypes.POINTER(c_api.NiteHandTrackerFrame)()
        c_api.niteReadHandTrackerFrame(self._handle, ctypes.byref(pfrm))
        return HandTrackerFrame(self._handle, pfrm)

    def set_smoothing_factor(self, factor):
        c_api.niteSetHandSmoothingFactor(self._handle, factor)

    def get_smoothing_factor(self):
        factor = ctypes.c_float()
        c_api.niteGetHandSmoothingFactor(self._handle, ctypes.byref(factor))
        return factor.value
    smoothing_factor = property(get_smoothing_factor, set_smoothing_factor)

    def start_hand_tracking(self, *position):
        new_hand_id = HandId()
        if len(position) == 3:
            position = Point3f(*position)
        elif len(position) == 1:
            position = position[0]
        else:
            raise TypeError("Either Point3f or three values required")
        c_api.niteStartHandTracking(self._handle, ctypes.byref(position), ctypes.byref(new_hand_id))
        return new_hand_id

    def stop_hand_tracking(self, handid):
        c_api.niteStopHandTracking(self._handle, handid)

    def start_gesture_detection(self, gesture_type):
        c_api.niteStartGestureDetection(self._handle, gesture_type)

    def stop_gesture_detection(self, gesture_type):
        c_api.niteStopGestureDetection(self._handle, gesture_type)

    def convert_hand_coordinates_to_depth(self, x, y, z):
        outX = ctypes.c_float()
        outY = ctypes.c_float()
        c_api.niteConvertHandCoordinatesToDepth(self._handle, x, y, z, ctypes.byref(outX), ctypes.byref(outY))
        return outX.value, outY.value

    def convert_depth_coordinates_to_hand(self, x, y, z):
        outX = ctypes.c_float()
        outY = ctypes.c_float()
        c_api.niteConvertDepthCoordinatesToHand(self._handle, x, y, z, ctypes.byref(outX), ctypes.byref(outY))
        return outX.value, outY.value

    def stop_all_hand_tracking(self):
        c_api.niteStopAllHandTracking(self._handle)

    def stop_all_gesture_detection(self):
        c_api.niteStopAllGestureDetection(self._handle)


class UserTrackerListener(HandleObject):
    def __init__(self, user_tracker):
        self.user_tracker = user_tracker
        self._callbacks = c_api.NiteUserTrackerCallbacks(
            readyForNextFrame=c_api.OniGeneralCallback(self._on_ready_for_next_frame))
        handle = ctypes.pointer(self._callbacks)
        c_api.niteRegisterUserTrackerCallbacks(self.user_tracker._handle, handle, None)
        HandleObject.__init__(self, handle)
        _registered_user_tracker_listeners.add(self)

    def unregister(self):
        self.close()

    def _close(self):
        if is_initialized():
            c_api.niteUnregisterUserTrackerCallbacks(self.user_tracker._handle, self._handle)
            self.user_tracker = None

    def _on_ready_for_next_frame(self, _):
        self.on_ready_for_next_frame()

    def on_ready_for_next_frame(self):
        """Implement me"""
        pass


class HandTrackerListener(HandleObject):
    def __init__(self, hand_tracker):
        self.hand_tracker = hand_tracker
        self._callbacks = c_api.NiteHandTrackerCallbacks(
            readyForNextFrame=c_api.OniGeneralCallback(self._on_ready_for_next_frame))
        handle = ctypes.pointer(self._callbacks)
        c_api.niteRegisterHandTrackerCallbacks(self.hand_tracker._handle, handle, None)
        HandleObject.__init__(self, handle)
        _registered_hand_tracker_listeners.add(self)

    def _close(self):
        if is_initialized():
            c_api.niteUnregisterHandTrackerCallbacks(self.hand_tracker._handle, self._handle)
            self.hand_tracker = None

    def _on_ready_for_next_frame(self, _):
        self.on_ready_for_next_frame()

    def on_ready_for_next_frame(self):
        """Implement me"""
        pass
