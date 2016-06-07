#!/usr/bin/env python
import os

try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

setup(name = "primesense",
    version = "2.2.0.30-5",
    description = "OpenNI2 and NiTE2 python bindings",
    author = "PrimeSense Inc",
    author_email = "primesense.com",
    license = "MIT",
    url = "http://www.openni.org/",
    packages = ["primesense"],
    platforms = ["POSIX", "Windows"],
    provides = ["primesense"],
    keywords = "PrimeSense, OpenNI, OpenNI2, Natural Interaction, NiTE, NiTE2",
    long_description = """\
Official `PrimeSense <http://www.primesense.com/>`_ Python-bindings for `OpenNI2 <http://www.openni.org/>`_
and `NiTE2 <http://www.openni.org/files/nite/>`_.

This package provides only the Python bindings; be sure to install OpenNI (and optionally NiTE) first.

Example::
    
    from primesense import openni2
    
    openni2.initialize()     # can also accept the path of the OpenNI redistribution
    
    dev = openni2.Device.open_any()
    print dev.get_sensor_info()
    
    depth_stream = dev.create_depth_stream()
    depth_stream.start()
    frame = depth_stream.read_frame()
    frame_data = frame.get_buffer_as_uint16()
    depth_stream.stop()
    
    openni2.unload()


.. note:: Refer to the C API for documentation

""",
    classifiers = [
        "Development Status :: 4 - Beta",
        "License :: OSI Approved :: MIT License",
        "Operating System :: Microsoft :: Windows",
        "Operating System :: POSIX",
        "Operating System :: MacOS",
        "Programming Language :: Python :: 2.6",
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.2",
        "Programming Language :: Python :: 3.3",
    ],
)

