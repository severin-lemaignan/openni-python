OpenNI2/NiTE2 Python Bindings
=============================

Python bindings for [OpenNI2](https://github.com/OpenNI/OpenNI2) and
[NiTE2](http://openni.ru/files/nite/).

Based on [Primesense original
bindings](https://pypi.python.org/pypi/primesense/2.2.0.30-5), with the bugs
(well, at least some of them) fixed.

Example of NiTE2 usage:

```python
import sys
from primesense import openni2, nite2, utils

openni2.initialize()
nite2.initialize()

dev = openni2.Device.open_any()

try:
    userTracker = nite2.UserTracker(dev)
except utils.NiteError as ne:
    logger.error("Unable to start the NiTE human tracker. Check "
                 "the error messages in the console. Model data "
                 "(s.dat, h.dat...) might be inaccessible.")
    sys.exit(-1)

while True:

    frame = userTracker.read_frame()

    if frame.users:
        for user in frame.users:
            if user.is_new():
                print("New human detected! Calibrating...")
                userTracker.start_skeleton_tracking(user.id)
            elif user.skeleton.state == nite2.SkeletonState.NITE_SKELETON_TRACKED:
                head = user.skeleton.joints[nite2.JointType.NITE_JOINT_HEAD]

                confidence = head.positionConfidence
                print("Head: (x:%dmm, y:%dmm, z:%dmm), confidence: %.2f" % (
                                                                    head.position.x,
                                                                    head.position.y,
                                                                    head.position.z,
                                                                    confidence))

nite2.unload()
openni2.unload()
```
