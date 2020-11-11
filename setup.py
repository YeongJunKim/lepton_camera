#! /usr/bin/env python2


import os
import sys

print(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['lepton_camera'],
    package_dir={'': 'src'},
    )

setup(**d)