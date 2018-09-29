# Auto-generated file; do not edit directly
# Tue Nov 12 13:59:13 2013
import sys

import ctypes
from openni.utils import CEnum, UnloadedDLL

TRUE = 1
FALSE = 0
ONI_MAX_STR = 256
ONI_MAX_SENSORS = 10
NITE_JOINT_COUNT = 15
NITE_POSE_COUNT = 2
NITE_VERSION_MAJOR = 2
NITE_VERSION_MINOR = 3
NITE_VERSION_MAINTENANCE = 0
NITE_VERSION_BUILD = 16
NITE_VERSION = (NITE_VERSION_MAJOR * 100000000 + NITE_VERSION_MINOR *
                1000000 + NITE_VERSION_MAINTENANCE * 10000 + NITE_VERSION_BUILD)


def _get_calling_conv(*args):
    if sys.platform == 'win32':
        return ctypes.WINFUNCTYPE(*args)
    else:
        return ctypes.CFUNCTYPE(*args)


from openni._openni2 import OniStatus
from openni._openni2 import OniSensorType
from openni._openni2 import OniPixelFormat
from openni._openni2 import OniDeviceState
from openni._openni2 import OniImageRegistrationMode


class _anon_enum_5(CEnum):
    _names_ = {'ONI_TIMEOUT_NONE': 0, 'ONI_TIMEOUT_FOREVER': -1}
    _values_ = {0: 'ONI_TIMEOUT_NONE', -1: 'ONI_TIMEOUT_FOREVER'}
    ONI_TIMEOUT_NONE = 0
    ONI_TIMEOUT_FOREVER = -1


ONI_TIMEOUT_NONE = _anon_enum_5.ONI_TIMEOUT_NONE
ONI_TIMEOUT_FOREVER = _anon_enum_5.ONI_TIMEOUT_FOREVER

from openni._openni2 import OniCallbackHandleImpl
from openni._openni2 import OniVersion
from openni._openni2 import OniVideoMode
from openni._openni2 import OniSensorInfo
from openni._openni2 import OniDeviceInfo


class _OniDevice(ctypes.Structure):

    def __repr__(self):
        return '_OniDevice()' % ()


class _OniStream(ctypes.Structure):

    def __repr__(self):
        return '_OniStream()' % ()


class _OniRecorder(ctypes.Structure):

    def __repr__(self):
        return '_OniRecorder()' % ()


from openni._openni2 import OniFrame
from openni._openni2 import OniDeviceCallbacks
from openni._openni2 import OniCropping
from openni._openni2 import OniRGB888Pixel
from openni._openni2 import OniYUV422DoublePixel
from openni._openni2 import OniSeek


class NiteJointType(CEnum):
    _names_ = {'NITE_JOINT_RIGHT_ELBOW': 5,
               'NITE_JOINT_LEFT_ELBOW': 4,
               'NITE_JOINT_RIGHT_KNEE': 12,
               'NITE_JOINT_RIGHT_HAND': 7,
               'NITE_JOINT_RIGHT_SHOULDER': 3,
               'NITE_JOINT_HEAD': 0,
               'NITE_JOINT_LEFT_HAND': 6,
               'NITE_JOINT_LEFT_SHOULDER': 2,
               'NITE_JOINT_LEFT_KNEE': 11,
               'NITE_JOINT_TORSO': 8,
               'NITE_JOINT_NECK': 1,
               'NITE_JOINT_LEFT_HIP': 9,
               'NITE_JOINT_RIGHT_HIP': 10,
               'NITE_JOINT_LEFT_FOOT': 13,
               'NITE_JOINT_RIGHT_FOOT': 14}
    _values_ = {0: 'NITE_JOINT_HEAD',
                1: 'NITE_JOINT_NECK',
                2: 'NITE_JOINT_LEFT_SHOULDER',
                3: 'NITE_JOINT_RIGHT_SHOULDER',
                4: 'NITE_JOINT_LEFT_ELBOW',
                5: 'NITE_JOINT_RIGHT_ELBOW',
                6: 'NITE_JOINT_LEFT_HAND',
                7: 'NITE_JOINT_RIGHT_HAND',
                8: 'NITE_JOINT_TORSO',
                9: 'NITE_JOINT_LEFT_HIP',
                10: 'NITE_JOINT_RIGHT_HIP',
                11: 'NITE_JOINT_LEFT_KNEE',
                12: 'NITE_JOINT_RIGHT_KNEE',
                13: 'NITE_JOINT_LEFT_FOOT',
                14: 'NITE_JOINT_RIGHT_FOOT'}
    NITE_JOINT_HEAD = 0
    NITE_JOINT_NECK = 1
    NITE_JOINT_LEFT_SHOULDER = 2
    NITE_JOINT_RIGHT_SHOULDER = 3
    NITE_JOINT_LEFT_ELBOW = 4
    NITE_JOINT_RIGHT_ELBOW = 5
    NITE_JOINT_LEFT_HAND = 6
    NITE_JOINT_RIGHT_HAND = 7
    NITE_JOINT_TORSO = 8
    NITE_JOINT_LEFT_HIP = 9
    NITE_JOINT_RIGHT_HIP = 10
    NITE_JOINT_LEFT_KNEE = 11
    NITE_JOINT_RIGHT_KNEE = 12
    NITE_JOINT_LEFT_FOOT = 13
    NITE_JOINT_RIGHT_FOOT = 14


class NiteSkeletonState(CEnum):
    _names_ = {'NITE_SKELETON_CALIBRATION_ERROR_HEAD': 5,
               'NITE_SKELETON_CALIBRATION_ERROR_NOT_IN_POSE': 3,
               'NITE_SKELETON_TRACKED': 2,
               'NITE_SKELETON_CALIBRATION_ERROR_LEGS': 6,
               'NITE_SKELETON_CALIBRATION_ERROR_HANDS': 4,
               'NITE_SKELETON_CALIBRATING': 1,
               'NITE_SKELETON_CALIBRATION_ERROR_TORSO': 7,
               'NITE_SKELETON_NONE': 0}
    _values_ = {0: 'NITE_SKELETON_NONE',
                1: 'NITE_SKELETON_CALIBRATING',
                2: 'NITE_SKELETON_TRACKED',
                3: 'NITE_SKELETON_CALIBRATION_ERROR_NOT_IN_POSE',
                4: 'NITE_SKELETON_CALIBRATION_ERROR_HANDS',
                5: 'NITE_SKELETON_CALIBRATION_ERROR_HEAD',
                6: 'NITE_SKELETON_CALIBRATION_ERROR_LEGS',
                7: 'NITE_SKELETON_CALIBRATION_ERROR_TORSO'}
    NITE_SKELETON_NONE = 0
    NITE_SKELETON_CALIBRATING = 1
    NITE_SKELETON_TRACKED = 2
    NITE_SKELETON_CALIBRATION_ERROR_NOT_IN_POSE = 3
    NITE_SKELETON_CALIBRATION_ERROR_HANDS = 4
    NITE_SKELETON_CALIBRATION_ERROR_HEAD = 5
    NITE_SKELETON_CALIBRATION_ERROR_LEGS = 6
    NITE_SKELETON_CALIBRATION_ERROR_TORSO = 7


class NiteUserState(CEnum):
    _names_ = {'NITE_USER_STATE_VISIBLE': 1, 'NITE_USER_STATE_NEW': 3, 'NITE_USER_STATE_LOST': 4}
    _values_ = {1: 'NITE_USER_STATE_VISIBLE', 3: 'NITE_USER_STATE_NEW', 4: 'NITE_USER_STATE_LOST'}
    NITE_USER_STATE_VISIBLE = 1
    NITE_USER_STATE_NEW = 3
    NITE_USER_STATE_LOST = 4


class NiteStatus(CEnum):
    _names_ = {'NITE_STATUS_OUT_OF_FLOW': 3, 'NITE_STATUS_ERROR': 1, 'NITE_STATUS_BAD_USER_ID': 2, 'NITE_STATUS_OK': 0}
    _values_ = {0: 'NITE_STATUS_OK', 1: 'NITE_STATUS_ERROR', 2: 'NITE_STATUS_BAD_USER_ID', 3: 'NITE_STATUS_OUT_OF_FLOW'}
    NITE_STATUS_OK = 0
    NITE_STATUS_ERROR = 1
    NITE_STATUS_BAD_USER_ID = 2
    NITE_STATUS_OUT_OF_FLOW = 3


class NitePoseType(CEnum):
    _names_ = {'NITE_POSE_CROSSED_HANDS': 1, 'NITE_POSE_PSI': 0}
    _values_ = {0: 'NITE_POSE_PSI', 1: 'NITE_POSE_CROSSED_HANDS'}
    NITE_POSE_PSI = 0
    NITE_POSE_CROSSED_HANDS = 1


class NitePoseState(CEnum):
    _names_ = {'NITE_POSE_STATE_IN_POSE': 2, 'NITE_POSE_STATE_ENTER': 4,
               'NITE_POSE_STATE_DETECTING': 1, 'NITE_POSE_STATE_EXIT': 8}
    _values_ = {8: 'NITE_POSE_STATE_EXIT', 1: 'NITE_POSE_STATE_DETECTING',
                2: 'NITE_POSE_STATE_IN_POSE', 4: 'NITE_POSE_STATE_ENTER'}
    NITE_POSE_STATE_DETECTING = 1
    NITE_POSE_STATE_IN_POSE = 2
    NITE_POSE_STATE_ENTER = 4
    NITE_POSE_STATE_EXIT = 8


class NiteGestureType(CEnum):
    _names_ = {'NITE_GESTURE_HAND_RAISE': 2, 'NITE_GESTURE_CLICK': 1, 'NITE_GESTURE_WAVE': 0}
    _values_ = {0: 'NITE_GESTURE_WAVE', 1: 'NITE_GESTURE_CLICK', 2: 'NITE_GESTURE_HAND_RAISE'}
    NITE_GESTURE_WAVE = 0
    NITE_GESTURE_CLICK = 1
    NITE_GESTURE_HAND_RAISE = 2


class NiteGestureState(CEnum):
    _names_ = {'NITE_GESTURE_STATE_IN_PROGRESS': 2, 'NITE_GESTURE_STATE_COMPLETED': 4, 'NITE_GESTURE_STATE_NEW': 1}
    _values_ = {1: 'NITE_GESTURE_STATE_NEW', 2: 'NITE_GESTURE_STATE_IN_PROGRESS', 4: 'NITE_GESTURE_STATE_COMPLETED'}
    NITE_GESTURE_STATE_NEW = 1
    NITE_GESTURE_STATE_IN_PROGRESS = 2
    NITE_GESTURE_STATE_COMPLETED = 4


class NiteHandState(CEnum):
    _names_ = {'NITE_HAND_STATE_NEW': 1, 'NITE_HAND_STATE_TRACKED': 2,
               'NITE_HAND_STATE_LOST': 0, 'NITE_HAND_STATE_TOUCHING_FOV': 4}
    _values_ = {0: 'NITE_HAND_STATE_LOST', 1: 'NITE_HAND_STATE_NEW',
                2: 'NITE_HAND_STATE_TRACKED', 4: 'NITE_HAND_STATE_TOUCHING_FOV'}
    NITE_HAND_STATE_LOST = 0
    NITE_HAND_STATE_NEW = 1
    NITE_HAND_STATE_TRACKED = 2
    NITE_HAND_STATE_TOUCHING_FOV = 4


class NiteUserTracker(ctypes.Structure):

    def __repr__(self):
        return 'NiteUserTracker()' % ()


class NitePoint3f(ctypes.Structure):
    x = 'ctypes.c_float'
    y = 'ctypes.c_float'
    z = 'ctypes.c_float'

    def __repr__(self):
        return 'NitePoint3f(x = %r, y = %r, z = %r)' % (self.x, self.y, self.z)


class NiteQuaternion(ctypes.Structure):
    x = 'ctypes.c_float'
    y = 'ctypes.c_float'
    z = 'ctypes.c_float'
    w = 'ctypes.c_float'

    def __repr__(self):
        return 'NiteQuaternion(x = %r, y = %r, z = %r, w = %r)' % (self.x, self.y, self.z, self.w)


class NiteSkeletonJoint(ctypes.Structure):
    jointType = 'NiteJointType'
    position = 'NitePoint3f'
    positionConfidence = 'ctypes.c_float'
    orientation = 'NiteQuaternion'
    orientationConfidence = 'ctypes.c_float'

    def __repr__(self):
        return 'NiteSkeletonJoint(jointType = %r, position = %r, positionConfidence = %r, orientation = %r, orientationConfidence = %r)' % (self.jointType, self.position, self.positionConfidence, self.orientation, self.orientationConfidence)


class NiteBoundingBox(ctypes.Structure):
    min = 'NitePoint3f'
    max = 'NitePoint3f'

    def __repr__(self):
        return 'NiteBoundingBox(min = %r, max = %r)' % (self.min, self.max)


class NitePoseData(ctypes.Structure):
    type = 'NitePoseType'
    state = 'ctypes.c_int'

    def __repr__(self):
        return 'NitePoseData(type = %r, state = %r)' % (self.type, self.state)


class NiteSkeleton(ctypes.Structure):
    joints = '(NiteSkeletonJoint * 15)'
    state = 'NiteSkeletonState'
    internalState = 'ctypes.c_int'

    def __repr__(self):
        return 'NiteSkeleton(joints = %r, state = %r, internalState = %r)' % (self.joints, self.state, self.internalState)


class NiteUserData(ctypes.Structure):
    id = 'NiteUserId'
    boundingBox = 'NiteBoundingBox'
    centerOfMass = 'NitePoint3f'
    state = 'ctypes.c_int'
    skeleton = 'NiteSkeleton'
    poses = '(NitePoseData * 2)'

    def __repr__(self):
        return 'NiteUserData(id = %r, boundingBox = %r, centerOfMass = %r, state = %r, skeleton = %r, poses = %r)' % (self.id, self.boundingBox, self.centerOfMass, self.state, self.skeleton, self.poses)


class NiteUserMap(ctypes.Structure):
    pixels = 'ctypes.POINTER(NiteUserId)'
    width = 'ctypes.c_int'
    height = 'ctypes.c_int'
    stride = 'ctypes.c_int'

    def __repr__(self):
        return 'NiteUserMap(pixels = %r, width = %r, height = %r, stride = %r)' % (self.pixels, self.width, self.height, self.stride)


class NitePlane(ctypes.Structure):
    point = 'NitePoint3f'
    normal = 'NitePoint3f'

    def __repr__(self):
        return 'NitePlane(point = %r, normal = %r)' % (self.point, self.normal)


class NiteUserTrackerFrame(ctypes.Structure):
    userCount = 'ctypes.c_int'
    pUser = 'ctypes.POINTER(NiteUserData)'
    userMap = 'NiteUserMap'
    pDepthFrame = 'ctypes.POINTER(OniFrame)'
    timestamp = 'ctypes.c_ulonglong'
    frameIndex = 'ctypes.c_int'
    floorConfidence = 'ctypes.c_float'
    floor = 'NitePlane'

    def __repr__(self):
        return 'NiteUserTrackerFrame(userCount = %r, pUser = %r, userMap = %r, pDepthFrame = %r, timestamp = %r, frameIndex = %r, floorConfidence = %r, floor = %r)' % (self.userCount, self.pUser, self.userMap, self.pDepthFrame, self.timestamp, self.frameIndex, self.floorConfidence, self.floor)


class NiteUserTrackerCallbacks(ctypes.Structure):
    readyForNextFrame = 'OniGeneralCallback'

    def __repr__(self):
        return 'NiteUserTrackerCallbacks(readyForNextFrame = %r)' % (self.readyForNextFrame)


class NiteHandData(ctypes.Structure):
    id = 'NiteHandId'
    position = 'NitePoint3f'
    state = 'ctypes.c_int'

    def __repr__(self):
        return 'NiteHandData(id = %r, position = %r, state = %r)' % (self.id, self.position, self.state)


class NiteGestureData(ctypes.Structure):
    type = 'NiteGestureType'
    currentPosition = 'NitePoint3f'
    state = 'ctypes.c_int'

    def __repr__(self):
        return 'NiteGestureData(type = %r, currentPosition = %r, state = %r)' % (self.type, self.currentPosition, self.state)


class NiteHandTrackerFrame(ctypes.Structure):
    handCount = 'ctypes.c_int'
    pHands = 'ctypes.POINTER(NiteHandData)'
    gestureCount = 'ctypes.c_int'
    pGestures = 'ctypes.POINTER(NiteGestureData)'
    pDepthFrame = 'ctypes.POINTER(OniFrame)'
    timestamp = 'ctypes.c_ulonglong'
    frameIndex = 'ctypes.c_int'

    def __repr__(self):
        return 'NiteHandTrackerFrame(handCount = %r, pHands = %r, gestureCount = %r, pGestures = %r, pDepthFrame = %r, timestamp = %r, frameIndex = %r)' % (self.handCount, self.pHands, self.gestureCount, self.pGestures, self.pDepthFrame, self.timestamp, self.frameIndex)


class NiteHandTrackerCallbacks(ctypes.Structure):
    readyForNextFrame = 'OniGeneralCallback'

    def __repr__(self):
        return 'NiteHandTrackerCallbacks(readyForNextFrame = %r)' % (self.readyForNextFrame)


class NiteVersion(ctypes.Structure):
    major = 'ctypes.c_int'
    minor = 'ctypes.c_int'
    maintenance = 'ctypes.c_int'
    build = 'ctypes.c_int'

    def __repr__(self):
        return 'NiteVersion(major = %r, minor = %r, maintenance = %r, build = %r)' % (self.major, self.minor, self.maintenance, self.build)


class NiteHandTracker(ctypes.Structure):

    def __repr__(self):
        return 'NiteHandTracker()' % ()


OniBool = ctypes.c_int
OniCallbackHandle = ctypes.POINTER(OniCallbackHandleImpl)
OniHardwareVersion = ctypes.c_int
OniDeviceHandle = ctypes.POINTER(_OniDevice)
OniStreamHandle = ctypes.POINTER(_OniStream)
OniRecorderHandle = ctypes.POINTER(_OniRecorder)
OniNewFrameCallback = _get_calling_conv(None, OniStreamHandle, ctypes.c_void_p)
OniGeneralCallback = _get_calling_conv(None, ctypes.c_void_p)
OniDeviceInfoCallback = _get_calling_conv(None, ctypes.POINTER(OniDeviceInfo), ctypes.c_void_p)
OniDeviceStateCallback = _get_calling_conv(None, ctypes.POINTER(OniDeviceInfo), OniDeviceState, ctypes.c_void_p)
OniFrameAllocBufferCallback = _get_calling_conv(ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p)
OniFrameFreeBufferCallback = _get_calling_conv(None, ctypes.c_void_p, ctypes.c_void_p)
OniDepthPixel = ctypes.c_ushort
OniGrayscale16Pixel = ctypes.c_ushort
OniGrayscale8Pixel = ctypes.c_ubyte
NiteUserId = ctypes.c_short
NiteUserTrackerHandle = ctypes.POINTER(NiteUserTracker)
NiteHandId = ctypes.c_short
NiteHandTrackerHandle = ctypes.POINTER(NiteHandTracker)

_OniDevice._fields_ = [
]

_OniStream._fields_ = [
]

_OniRecorder._fields_ = [
]

NiteUserTracker._fields_ = [
]

NitePoint3f._fields_ = [
    ('x', ctypes.c_float),
    ('y', ctypes.c_float),
    ('z', ctypes.c_float),
]

NiteQuaternion._fields_ = [
    ('x', ctypes.c_float),
    ('y', ctypes.c_float),
    ('z', ctypes.c_float),
    ('w', ctypes.c_float),
]

NiteSkeletonJoint._fields_ = [
    ('jointType', NiteJointType),
    ('position', NitePoint3f),
    ('positionConfidence', ctypes.c_float),
    ('orientation', NiteQuaternion),
    ('orientationConfidence', ctypes.c_float),
]

NiteBoundingBox._fields_ = [
    ('min', NitePoint3f),
    ('max', NitePoint3f),
]

NitePoseData._fields_ = [
    ('type', NitePoseType),
    ('state', ctypes.c_int),
]

NiteSkeleton._fields_ = [
    ('joints', (NiteSkeletonJoint * 15)),
    ('state', NiteSkeletonState),
    ('internalState', ctypes.c_int),
]

NiteUserData._fields_ = [
    ('id', NiteUserId),
    ('boundingBox', NiteBoundingBox),
    ('centerOfMass', NitePoint3f),
    ('state', ctypes.c_int),
    ('skeleton', NiteSkeleton),
    ('poses', (NitePoseData * 2)),
]

NiteUserMap._fields_ = [
    ('pixels', ctypes.POINTER(NiteUserId)),
    ('width', ctypes.c_int),
    ('height', ctypes.c_int),
    ('stride', ctypes.c_int),
]

NitePlane._fields_ = [
    ('point', NitePoint3f),
    ('normal', NitePoint3f),
]

NiteUserTrackerFrame._fields_ = [
    ('userCount', ctypes.c_int),
    ('pUser', ctypes.POINTER(NiteUserData)),
    ('userMap', NiteUserMap),
    ('pDepthFrame', ctypes.POINTER(OniFrame)),
    ('timestamp', ctypes.c_ulonglong),
    ('frameIndex', ctypes.c_int),
    ('floorConfidence', ctypes.c_float),
    ('floor', NitePlane),
]

NiteUserTrackerCallbacks._fields_ = [
    ('readyForNextFrame', OniGeneralCallback),
]

NiteHandData._fields_ = [
    ('id', NiteHandId),
    ('position', NitePoint3f),
    ('state', ctypes.c_int),
]

NiteGestureData._fields_ = [
    ('type', NiteGestureType),
    ('currentPosition', NitePoint3f),
    ('state', ctypes.c_int),
]

NiteHandTrackerFrame._fields_ = [
    ('handCount', ctypes.c_int),
    ('pHands', ctypes.POINTER(NiteHandData)),
    ('gestureCount', ctypes.c_int),
    ('pGestures', ctypes.POINTER(NiteGestureData)),
    ('pDepthFrame', ctypes.POINTER(OniFrame)),
    ('timestamp', ctypes.c_ulonglong),
    ('frameIndex', ctypes.c_int),
]

NiteHandTrackerCallbacks._fields_ = [
    ('readyForNextFrame', OniGeneralCallback),
]

NiteVersion._fields_ = [
    ('major', ctypes.c_int),
    ('minor', ctypes.c_int),
    ('maintenance', ctypes.c_int),
    ('build', ctypes.c_int),
]

NiteHandTracker._fields_ = [
]

_the_dll = UnloadedDLL
_niteInitialize = UnloadedDLL
_niteShutdown = UnloadedDLL
_niteGetVersion = UnloadedDLL
_niteShutdownUserTracker = UnloadedDLL
_niteStartSkeletonTracking = UnloadedDLL
_niteStopSkeletonTracking = UnloadedDLL
_niteIsSkeletonTracking = UnloadedDLL
_niteSetSkeletonSmoothing = UnloadedDLL
_niteGetSkeletonSmoothing = UnloadedDLL
_niteStartPoseDetection = UnloadedDLL
_niteStopPoseDetection = UnloadedDLL
_niteStopAllPoseDetection = UnloadedDLL
_niteRegisterUserTrackerCallbacks = UnloadedDLL
_niteUnregisterUserTrackerCallbacks = UnloadedDLL
_niteReadUserTrackerFrame = UnloadedDLL
_niteUserTrackerFrameAddRef = UnloadedDLL
_niteUserTrackerFrameRelease = UnloadedDLL
_niteShutdownHandTracker = UnloadedDLL
_niteStartHandTracking = UnloadedDLL
_niteStopHandTracking = UnloadedDLL
_niteStopAllHandTracking = UnloadedDLL
_niteSetHandSmoothingFactor = UnloadedDLL
_niteGetHandSmoothingFactor = UnloadedDLL
_niteRegisterHandTrackerCallbacks = UnloadedDLL
_niteUnregisterHandTrackerCallbacks = UnloadedDLL
_niteReadHandTrackerFrame = UnloadedDLL
_niteHandTrackerFrameAddRef = UnloadedDLL
_niteHandTrackerFrameRelease = UnloadedDLL
_niteStartGestureDetection = UnloadedDLL
_niteStopGestureDetection = UnloadedDLL
_niteStopAllGestureDetection = UnloadedDLL
_niteConvertJointCoordinatesToDepth = UnloadedDLL
_niteConvertDepthCoordinatesToJoint = UnloadedDLL
_niteConvertHandCoordinatesToDepth = UnloadedDLL
_niteConvertDepthCoordinatesToHand = UnloadedDLL
_niteInitializeUserTracker = UnloadedDLL
_niteInitializeUserTrackerByDevice = UnloadedDLL
_niteInitializeHandTracker = UnloadedDLL
_niteInitializeHandTrackerByDevice = UnloadedDLL


def load_dll(dllname):
    global _the_dll
    if _the_dll:
        raise ValueError('DLL already loaded')
    dll = ctypes.CDLL(dllname)

    global _niteInitialize
    _niteInitialize = dll.niteInitialize
    _niteInitialize.restype = NiteStatus
    _niteInitialize.argtypes = []

    global _niteShutdown
    _niteShutdown = dll.niteShutdown
    _niteShutdown.restype = None
    _niteShutdown.argtypes = []

    global _niteGetVersion
    _niteGetVersion = dll.niteGetVersion
    _niteGetVersion.restype = NiteVersion
    _niteGetVersion.argtypes = []

    global _niteShutdownUserTracker
    _niteShutdownUserTracker = dll.niteShutdownUserTracker
    _niteShutdownUserTracker.restype = NiteStatus
    _niteShutdownUserTracker.argtypes = [NiteUserTrackerHandle]

    global _niteStartSkeletonTracking
    _niteStartSkeletonTracking = dll.niteStartSkeletonTracking
    _niteStartSkeletonTracking.restype = NiteStatus
    _niteStartSkeletonTracking.argtypes = [NiteUserTrackerHandle, NiteUserId]

    global _niteStopSkeletonTracking
    _niteStopSkeletonTracking = dll.niteStopSkeletonTracking
    _niteStopSkeletonTracking.restype = None
    _niteStopSkeletonTracking.argtypes = [NiteUserTrackerHandle, NiteUserId]

    global _niteIsSkeletonTracking
    _niteIsSkeletonTracking = dll.niteIsSkeletonTracking
    _niteIsSkeletonTracking.restype = ctypes.c_bool
    _niteIsSkeletonTracking.argtypes = [NiteUserTrackerHandle, NiteUserId]

    global _niteSetSkeletonSmoothing
    _niteSetSkeletonSmoothing = dll.niteSetSkeletonSmoothing
    _niteSetSkeletonSmoothing.restype = NiteStatus
    _niteSetSkeletonSmoothing.argtypes = [NiteUserTrackerHandle, ctypes.c_float]

    global _niteGetSkeletonSmoothing
    _niteGetSkeletonSmoothing = dll.niteGetSkeletonSmoothing
    _niteGetSkeletonSmoothing.restype = NiteStatus
    _niteGetSkeletonSmoothing.argtypes = [NiteUserTrackerHandle, ctypes.POINTER(ctypes.c_float)]

    global _niteStartPoseDetection
    _niteStartPoseDetection = dll.niteStartPoseDetection
    _niteStartPoseDetection.restype = NiteStatus
    _niteStartPoseDetection.argtypes = [NiteUserTrackerHandle, NiteUserId, NitePoseType]

    global _niteStopPoseDetection
    _niteStopPoseDetection = dll.niteStopPoseDetection
    _niteStopPoseDetection.restype = None
    _niteStopPoseDetection.argtypes = [NiteUserTrackerHandle, NiteUserId, NitePoseType]

    global _niteStopAllPoseDetection
    _niteStopAllPoseDetection = dll.niteStopAllPoseDetection
    _niteStopAllPoseDetection.restype = None
    _niteStopAllPoseDetection.argtypes = [NiteUserTrackerHandle, NiteUserId]

    global _niteRegisterUserTrackerCallbacks
    _niteRegisterUserTrackerCallbacks = dll.niteRegisterUserTrackerCallbacks
    _niteRegisterUserTrackerCallbacks.restype = NiteStatus
    _niteRegisterUserTrackerCallbacks.argtypes = [NiteUserTrackerHandle,
                                                  ctypes.POINTER(NiteUserTrackerCallbacks), ctypes.c_void_p]

    global _niteUnregisterUserTrackerCallbacks
    _niteUnregisterUserTrackerCallbacks = dll.niteUnregisterUserTrackerCallbacks
    _niteUnregisterUserTrackerCallbacks.restype = None
    _niteUnregisterUserTrackerCallbacks.argtypes = [NiteUserTrackerHandle, ctypes.POINTER(NiteUserTrackerCallbacks)]

    global _niteReadUserTrackerFrame
    _niteReadUserTrackerFrame = dll.niteReadUserTrackerFrame
    _niteReadUserTrackerFrame.restype = NiteStatus
    _niteReadUserTrackerFrame.argtypes = [NiteUserTrackerHandle, ctypes.POINTER(ctypes.POINTER(NiteUserTrackerFrame))]

    global _niteUserTrackerFrameAddRef
    _niteUserTrackerFrameAddRef = dll.niteUserTrackerFrameAddRef
    _niteUserTrackerFrameAddRef.restype = NiteStatus
    _niteUserTrackerFrameAddRef.argtypes = [NiteUserTrackerHandle, ctypes.POINTER(NiteUserTrackerFrame)]

    global _niteUserTrackerFrameRelease
    _niteUserTrackerFrameRelease = dll.niteUserTrackerFrameRelease
    _niteUserTrackerFrameRelease.restype = NiteStatus
    _niteUserTrackerFrameRelease.argtypes = [NiteUserTrackerHandle, ctypes.POINTER(NiteUserTrackerFrame)]

    global _niteShutdownHandTracker
    _niteShutdownHandTracker = dll.niteShutdownHandTracker
    _niteShutdownHandTracker.restype = NiteStatus
    _niteShutdownHandTracker.argtypes = [NiteHandTrackerHandle]

    global _niteStartHandTracking
    _niteStartHandTracking = dll.niteStartHandTracking
    _niteStartHandTracking.restype = NiteStatus
    _niteStartHandTracking.argtypes = [NiteHandTrackerHandle, ctypes.POINTER(NitePoint3f), ctypes.POINTER(NiteHandId)]

    global _niteStopHandTracking
    _niteStopHandTracking = dll.niteStopHandTracking
    _niteStopHandTracking.restype = None
    _niteStopHandTracking.argtypes = [NiteHandTrackerHandle, NiteHandId]

    global _niteStopAllHandTracking
    _niteStopAllHandTracking = dll.niteStopAllHandTracking
    _niteStopAllHandTracking.restype = None
    _niteStopAllHandTracking.argtypes = [NiteHandTrackerHandle]

    global _niteSetHandSmoothingFactor
    _niteSetHandSmoothingFactor = dll.niteSetHandSmoothingFactor
    _niteSetHandSmoothingFactor.restype = NiteStatus
    _niteSetHandSmoothingFactor.argtypes = [NiteHandTrackerHandle, ctypes.c_float]

    global _niteGetHandSmoothingFactor
    _niteGetHandSmoothingFactor = dll.niteGetHandSmoothingFactor
    _niteGetHandSmoothingFactor.restype = NiteStatus
    _niteGetHandSmoothingFactor.argtypes = [NiteHandTrackerHandle, ctypes.POINTER(ctypes.c_float)]

    global _niteRegisterHandTrackerCallbacks
    _niteRegisterHandTrackerCallbacks = dll.niteRegisterHandTrackerCallbacks
    _niteRegisterHandTrackerCallbacks.restype = NiteStatus
    _niteRegisterHandTrackerCallbacks.argtypes = [NiteHandTrackerHandle,
                                                  ctypes.POINTER(NiteHandTrackerCallbacks), ctypes.c_void_p]

    global _niteUnregisterHandTrackerCallbacks
    _niteUnregisterHandTrackerCallbacks = dll.niteUnregisterHandTrackerCallbacks
    _niteUnregisterHandTrackerCallbacks.restype = None
    _niteUnregisterHandTrackerCallbacks.argtypes = [NiteHandTrackerHandle, ctypes.POINTER(NiteHandTrackerCallbacks)]

    global _niteReadHandTrackerFrame
    _niteReadHandTrackerFrame = dll.niteReadHandTrackerFrame
    _niteReadHandTrackerFrame.restype = NiteStatus
    _niteReadHandTrackerFrame.argtypes = [NiteHandTrackerHandle, ctypes.POINTER(ctypes.POINTER(NiteHandTrackerFrame))]

    global _niteHandTrackerFrameAddRef
    _niteHandTrackerFrameAddRef = dll.niteHandTrackerFrameAddRef
    _niteHandTrackerFrameAddRef.restype = NiteStatus
    _niteHandTrackerFrameAddRef.argtypes = [NiteHandTrackerHandle, ctypes.POINTER(NiteHandTrackerFrame)]

    global _niteHandTrackerFrameRelease
    _niteHandTrackerFrameRelease = dll.niteHandTrackerFrameRelease
    _niteHandTrackerFrameRelease.restype = NiteStatus
    _niteHandTrackerFrameRelease.argtypes = [NiteHandTrackerHandle, ctypes.POINTER(NiteHandTrackerFrame)]

    global _niteStartGestureDetection
    _niteStartGestureDetection = dll.niteStartGestureDetection
    _niteStartGestureDetection.restype = NiteStatus
    _niteStartGestureDetection.argtypes = [NiteHandTrackerHandle, NiteGestureType]

    global _niteStopGestureDetection
    _niteStopGestureDetection = dll.niteStopGestureDetection
    _niteStopGestureDetection.restype = None
    _niteStopGestureDetection.argtypes = [NiteHandTrackerHandle, NiteGestureType]

    global _niteStopAllGestureDetection
    _niteStopAllGestureDetection = dll.niteStopAllGestureDetection
    _niteStopAllGestureDetection.restype = None
    _niteStopAllGestureDetection.argtypes = [NiteHandTrackerHandle]

    global _niteConvertJointCoordinatesToDepth
    _niteConvertJointCoordinatesToDepth = dll.niteConvertJointCoordinatesToDepth
    _niteConvertJointCoordinatesToDepth.restype = NiteStatus
    _niteConvertJointCoordinatesToDepth.argtypes = [NiteUserTrackerHandle, ctypes.c_float,
                                                    ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]

    global _niteConvertDepthCoordinatesToJoint
    _niteConvertDepthCoordinatesToJoint = dll.niteConvertDepthCoordinatesToJoint
    _niteConvertDepthCoordinatesToJoint.restype = NiteStatus
    _niteConvertDepthCoordinatesToJoint.argtypes = [
        NiteUserTrackerHandle, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]

    global _niteConvertHandCoordinatesToDepth
    _niteConvertHandCoordinatesToDepth = dll.niteConvertHandCoordinatesToDepth
    _niteConvertHandCoordinatesToDepth.restype = NiteStatus
    _niteConvertHandCoordinatesToDepth.argtypes = [NiteHandTrackerHandle, ctypes.c_float,
                                                   ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]

    global _niteConvertDepthCoordinatesToHand
    _niteConvertDepthCoordinatesToHand = dll.niteConvertDepthCoordinatesToHand
    _niteConvertDepthCoordinatesToHand.restype = NiteStatus
    _niteConvertDepthCoordinatesToHand.argtypes = [
        NiteHandTrackerHandle, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]

    global _niteInitializeUserTracker
    _niteInitializeUserTracker = dll.niteInitializeUserTracker
    _niteInitializeUserTracker.restype = NiteStatus
    _niteInitializeUserTracker.argtypes = [ctypes.POINTER(NiteUserTrackerHandle)]

    global _niteInitializeUserTrackerByDevice
    _niteInitializeUserTrackerByDevice = dll.niteInitializeUserTrackerByDevice
    _niteInitializeUserTrackerByDevice.restype = NiteStatus
    _niteInitializeUserTrackerByDevice.argtypes = [ctypes.c_void_p, ctypes.POINTER(NiteUserTrackerHandle)]

    global _niteInitializeHandTracker
    _niteInitializeHandTracker = dll.niteInitializeHandTracker
    _niteInitializeHandTracker.restype = NiteStatus
    _niteInitializeHandTracker.argtypes = [ctypes.POINTER(NiteHandTrackerHandle)]

    global _niteInitializeHandTrackerByDevice
    _niteInitializeHandTrackerByDevice = dll.niteInitializeHandTrackerByDevice
    _niteInitializeHandTrackerByDevice.restype = NiteStatus
    _niteInitializeHandTrackerByDevice.argtypes = [ctypes.c_void_p, ctypes.POINTER(NiteHandTrackerHandle)]

    _the_dll = dll


import functools
from openni.utils import NiteError


def nite_call(func):
    @functools.wraps(func)
    def wrapper(*args):
        res = func(*args)
        if res != NiteStatus.NITE_STATUS_OK:
            raise NiteError(res)
        return res

    return wrapper


@nite_call
def niteInitialize():
    '''NiteStatus niteInitialize()'''
    return _niteInitialize()


def niteShutdown():
    '''void niteShutdown()'''
    return _niteShutdown()


def niteGetVersion():
    '''NiteVersion niteGetVersion()'''
    return _niteGetVersion()


@nite_call
def niteShutdownUserTracker(userTracker):
    '''NiteStatus niteShutdownUserTracker(NiteUserTrackerHandle userTracker)'''
    return _niteShutdownUserTracker(userTracker)


@nite_call
def niteStartSkeletonTracking(userTracker, id):
    '''NiteStatus niteStartSkeletonTracking(NiteUserTrackerHandle userTracker, NiteUserId id)'''
    return _niteStartSkeletonTracking(userTracker, id)


def niteStopSkeletonTracking(userTracker, id):
    '''void niteStopSkeletonTracking(NiteUserTrackerHandle userTracker, NiteUserId id)'''
    return _niteStopSkeletonTracking(userTracker, id)


def niteIsSkeletonTracking(userTracker, id):
    '''bool niteIsSkeletonTracking(NiteUserTrackerHandle userTracker, NiteUserId id)'''
    return _niteIsSkeletonTracking(userTracker, id)


@nite_call
def niteSetSkeletonSmoothing(userTracker, factor):
    '''NiteStatus niteSetSkeletonSmoothing(NiteUserTrackerHandle userTracker, float factor)'''
    return _niteSetSkeletonSmoothing(userTracker, factor)


@nite_call
def niteGetSkeletonSmoothing(userTracker, pFactor):
    '''NiteStatus niteGetSkeletonSmoothing(NiteUserTrackerHandle userTracker, float* pFactor)'''
    return _niteGetSkeletonSmoothing(userTracker, pFactor)


@nite_call
def niteStartPoseDetection(userTracker, id, poseType):
    '''NiteStatus niteStartPoseDetection(NiteUserTrackerHandle userTracker, NiteUserId id, NitePoseType poseType)'''
    return _niteStartPoseDetection(userTracker, id, poseType)


def niteStopPoseDetection(userTracker, id, poseType):
    '''void niteStopPoseDetection(NiteUserTrackerHandle userTracker, NiteUserId id, NitePoseType poseType)'''
    return _niteStopPoseDetection(userTracker, id, poseType)


def niteStopAllPoseDetection(userTracker, id):
    '''void niteStopAllPoseDetection(NiteUserTrackerHandle userTracker, NiteUserId id)'''
    return _niteStopAllPoseDetection(userTracker, id)


@nite_call
def niteRegisterUserTrackerCallbacks(userTracker, pCallbacks, pCookie):
    '''NiteStatus niteRegisterUserTrackerCallbacks(NiteUserTrackerHandle userTracker, NiteUserTrackerCallbacks* pCallbacks, void* pCookie)'''
    return _niteRegisterUserTrackerCallbacks(userTracker, pCallbacks, pCookie)


def niteUnregisterUserTrackerCallbacks(userTracker, pCallbacks):
    '''void niteUnregisterUserTrackerCallbacks(NiteUserTrackerHandle userTracker, NiteUserTrackerCallbacks* pCallbacks)'''
    return _niteUnregisterUserTrackerCallbacks(userTracker, pCallbacks)


@nite_call
def niteReadUserTrackerFrame(userTracker, pUserTrackerFrame):
    '''NiteStatus niteReadUserTrackerFrame(NiteUserTrackerHandle userTracker, NiteUserTrackerFrame** pUserTrackerFrame)'''
    return _niteReadUserTrackerFrame(userTracker, pUserTrackerFrame)


@nite_call
def niteUserTrackerFrameAddRef(userTracker, pUserTrackerFrame):
    '''NiteStatus niteUserTrackerFrameAddRef(NiteUserTrackerHandle userTracker, NiteUserTrackerFrame* pUserTrackerFrame)'''
    return _niteUserTrackerFrameAddRef(userTracker, pUserTrackerFrame)


@nite_call
def niteUserTrackerFrameRelease(userTracker, pUserTrackerFrame):
    '''NiteStatus niteUserTrackerFrameRelease(NiteUserTrackerHandle userTracker, NiteUserTrackerFrame* pUserTrackerFrame)'''
    return _niteUserTrackerFrameRelease(userTracker, pUserTrackerFrame)


@nite_call
def niteShutdownHandTracker(handTracker):
    '''NiteStatus niteShutdownHandTracker(NiteHandTrackerHandle handTracker)'''
    return _niteShutdownHandTracker(handTracker)


@nite_call
def niteStartHandTracking(handTracker, pPosition, pNewHandId):
    '''NiteStatus niteStartHandTracking(NiteHandTrackerHandle handTracker, NitePoint3f* pPosition, NiteHandId* pNewHandId)'''
    return _niteStartHandTracking(handTracker, pPosition, pNewHandId)


def niteStopHandTracking(handTracker, id):
    '''void niteStopHandTracking(NiteHandTrackerHandle handTracker, NiteHandId id)'''
    return _niteStopHandTracking(handTracker, id)


def niteStopAllHandTracking(handTracker):
    '''void niteStopAllHandTracking(NiteHandTrackerHandle handTracker)'''
    return _niteStopAllHandTracking(handTracker)


@nite_call
def niteSetHandSmoothingFactor(handTracker, factor):
    '''NiteStatus niteSetHandSmoothingFactor(NiteHandTrackerHandle handTracker, float factor)'''
    return _niteSetHandSmoothingFactor(handTracker, factor)


@nite_call
def niteGetHandSmoothingFactor(handTracker, pFactor):
    '''NiteStatus niteGetHandSmoothingFactor(NiteHandTrackerHandle handTracker, float* pFactor)'''
    return _niteGetHandSmoothingFactor(handTracker, pFactor)


@nite_call
def niteRegisterHandTrackerCallbacks(handTracker, pCallbacks, pCookie):
    '''NiteStatus niteRegisterHandTrackerCallbacks(NiteHandTrackerHandle handTracker, NiteHandTrackerCallbacks* pCallbacks, void* pCookie)'''
    return _niteRegisterHandTrackerCallbacks(handTracker, pCallbacks, pCookie)


def niteUnregisterHandTrackerCallbacks(handTracker, pCallbacks):
    '''void niteUnregisterHandTrackerCallbacks(NiteHandTrackerHandle handTracker, NiteHandTrackerCallbacks* pCallbacks)'''
    return _niteUnregisterHandTrackerCallbacks(handTracker, pCallbacks)


@nite_call
def niteReadHandTrackerFrame(handTracker, pHandTrackerFrame):
    '''NiteStatus niteReadHandTrackerFrame(NiteHandTrackerHandle handTracker, NiteHandTrackerFrame** pHandTrackerFrame)'''
    return _niteReadHandTrackerFrame(handTracker, pHandTrackerFrame)


@nite_call
def niteHandTrackerFrameAddRef(handTracker, pHandTrackerFrame):
    '''NiteStatus niteHandTrackerFrameAddRef(NiteHandTrackerHandle handTracker, NiteHandTrackerFrame* pHandTrackerFrame)'''
    return _niteHandTrackerFrameAddRef(handTracker, pHandTrackerFrame)


@nite_call
def niteHandTrackerFrameRelease(handTracker, pHandTrackerFrame):
    '''NiteStatus niteHandTrackerFrameRelease(NiteHandTrackerHandle handTracker, NiteHandTrackerFrame* pHandTrackerFrame)'''
    return _niteHandTrackerFrameRelease(handTracker, pHandTrackerFrame)


@nite_call
def niteStartGestureDetection(handTracker, type):
    '''NiteStatus niteStartGestureDetection(NiteHandTrackerHandle handTracker, NiteGestureType type)'''
    return _niteStartGestureDetection(handTracker, type)


def niteStopGestureDetection(handTracker, type):
    '''void niteStopGestureDetection(NiteHandTrackerHandle handTracker, NiteGestureType type)'''
    return _niteStopGestureDetection(handTracker, type)


def niteStopAllGestureDetection(handTracker):
    '''void niteStopAllGestureDetection(NiteHandTrackerHandle handTracker)'''
    return _niteStopAllGestureDetection(handTracker)


@nite_call
def niteConvertJointCoordinatesToDepth(userTracker, x, y, z, pX, pY):
    '''NiteStatus niteConvertJointCoordinatesToDepth(NiteUserTrackerHandle userTracker, float x, float y, float z, float* pX, float* pY)'''
    return _niteConvertJointCoordinatesToDepth(userTracker, x, y, z, pX, pY)


@nite_call
def niteConvertDepthCoordinatesToJoint(userTracker, x, y, z, pX, pY):
    '''NiteStatus niteConvertDepthCoordinatesToJoint(NiteUserTrackerHandle userTracker, int x, int y, int z, float* pX, float* pY)'''
    return _niteConvertDepthCoordinatesToJoint(userTracker, x, y, z, pX, pY)


@nite_call
def niteConvertHandCoordinatesToDepth(handTracker, x, y, z, pX, pY):
    '''NiteStatus niteConvertHandCoordinatesToDepth(NiteHandTrackerHandle handTracker, float x, float y, float z, float* pX, float* pY)'''
    return _niteConvertHandCoordinatesToDepth(handTracker, x, y, z, pX, pY)


@nite_call
def niteConvertDepthCoordinatesToHand(handTracker, x, y, z, pX, pY):
    '''NiteStatus niteConvertDepthCoordinatesToHand(NiteHandTrackerHandle handTracker, int x, int y, int z, float* pX, float* pY)'''
    return _niteConvertDepthCoordinatesToHand(handTracker, x, y, z, pX, pY)


@nite_call
def niteInitializeUserTracker(pUserTracker):
    '''NiteStatus niteInitializeUserTracker(NiteUserTrackerHandle* pUserTracker)'''
    return _niteInitializeUserTracker(pUserTracker)


@nite_call
def niteInitializeUserTrackerByDevice(pDevice, pUserTracker):
    '''NiteStatus niteInitializeUserTrackerByDevice(void* pDevice, NiteUserTrackerHandle* pUserTracker)'''
    return _niteInitializeUserTrackerByDevice(pDevice, pUserTracker)


@nite_call
def niteInitializeHandTracker(pHandTracker):
    '''NiteStatus niteInitializeHandTracker(NiteHandTrackerHandle* pHandTracker)'''
    return _niteInitializeHandTracker(pHandTracker)


@nite_call
def niteInitializeHandTrackerByDevice(pDevice, pHandTracker):
    '''NiteStatus niteInitializeHandTrackerByDevice(void* pDevice, NiteHandTrackerHandle* pHandTracker)'''
    return _niteInitializeHandTrackerByDevice(pDevice, pHandTracker)


all_types = [
    OniStatus,
    OniSensorType,
    OniPixelFormat,
    OniDeviceState,
    OniImageRegistrationMode,
    _anon_enum_5,
    OniBool,
    OniCallbackHandleImpl,
    OniCallbackHandle,
    OniVersion,
    OniHardwareVersion,
    OniVideoMode,
    OniSensorInfo,
    OniDeviceInfo,
    _OniDevice,
    OniDeviceHandle,
    _OniStream,
    OniStreamHandle,
    _OniRecorder,
    OniRecorderHandle,
    OniFrame,
    OniNewFrameCallback,
    OniGeneralCallback,
    OniDeviceInfoCallback,
    OniDeviceStateCallback,
    OniFrameAllocBufferCallback,
    OniFrameFreeBufferCallback,
    OniDeviceCallbacks,
    OniCropping,
    OniDepthPixel,
    OniGrayscale16Pixel,
    OniGrayscale8Pixel,
    OniRGB888Pixel,
    OniYUV422DoublePixel,
    OniSeek,
    NiteJointType,
    NiteSkeletonState,
    NiteUserState,
    NiteStatus,
    NitePoseType,
    NitePoseState,
    NiteGestureType,
    NiteGestureState,
    NiteHandState,
    NiteUserId,
    NiteUserTracker,
    NiteUserTrackerHandle,
    NitePoint3f,
    NiteQuaternion,
    NiteSkeletonJoint,
    NiteBoundingBox,
    NitePoseData,
    NiteSkeleton,
    NiteUserData,
    NiteUserMap,
    NitePlane,
    NiteUserTrackerFrame,
    NiteUserTrackerCallbacks,
    NiteHandId,
    NiteHandData,
    NiteGestureData,
    NiteHandTrackerFrame,
    NiteHandTrackerCallbacks,
    NiteVersion,
    NiteHandTracker,
    NiteHandTrackerHandle,
]

all_funcs = [
    niteInitialize,
    niteShutdown,
    niteGetVersion,
    niteShutdownUserTracker,
    niteStartSkeletonTracking,
    niteStopSkeletonTracking,
    niteIsSkeletonTracking,
    niteSetSkeletonSmoothing,
    niteGetSkeletonSmoothing,
    niteStartPoseDetection,
    niteStopPoseDetection,
    niteStopAllPoseDetection,
    niteRegisterUserTrackerCallbacks,
    niteUnregisterUserTrackerCallbacks,
    niteReadUserTrackerFrame,
    niteUserTrackerFrameAddRef,
    niteUserTrackerFrameRelease,
    niteShutdownHandTracker,
    niteStartHandTracking,
    niteStopHandTracking,
    niteStopAllHandTracking,
    niteSetHandSmoothingFactor,
    niteGetHandSmoothingFactor,
    niteRegisterHandTrackerCallbacks,
    niteUnregisterHandTrackerCallbacks,
    niteReadHandTrackerFrame,
    niteHandTrackerFrameAddRef,
    niteHandTrackerFrameRelease,
    niteStartGestureDetection,
    niteStopGestureDetection,
    niteStopAllGestureDetection,
    niteConvertJointCoordinatesToDepth,
    niteConvertDepthCoordinatesToJoint,
    niteConvertHandCoordinatesToDepth,
    niteConvertDepthCoordinatesToHand,
    niteInitializeUserTracker,
    niteInitializeUserTrackerByDevice,
    niteInitializeHandTracker,
    niteInitializeHandTrackerByDevice,
]
