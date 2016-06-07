#!/usr/bin/env python
#-*- coding: UTF-8 -*-
import os

try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

setup(name = "openni",
    version = "2.2.0-6",
    description = "OpenNI2 and NiTE2 python bindings",
    author = "PrimeSense Inc, Séverin Lemaignan",
    author_email = "primesense.com, severin.lemaignan@plymouth.ac.uk",
    license = "MIT",
    url = "https://github.com/severin-lemaignan/openni-python",
    packages = ["openni"],
    platforms = ["POSIX", "Windows"],
    provides = ["openni"],
    keywords = "PrimeSense, OpenNI, OpenNI2, Natural Interaction, NiTE, NiTE2",
    long_description = """\
Python-bindings for `OpenNI2 <https://github.com/OpenNI/OpenNI2>`_
and `NiTE2 <http://openni.ru/files/nite/>`_.

This package provides only the Python bindings; be sure to install OpenNI2 (and optionally NiTE2) first.

Example::
    
    from openni import openni2
    
    openni2.initialize()     # can also accept the path of the OpenNI redistribution
    
    dev = openni2.Device.open_any()
    print dev.get_device_info()
    
    depth_stream = dev.create_depth_stream()
    depth_stream.start()
    frame = depth_stream.read_frame()
    frame_data = frame.get_buffer_as_uint16()
    depth_stream.stop()
    
    openni2.unload()


.. note:: Refer to the OpenNI2/NiTE2 C API for complete documentation

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

