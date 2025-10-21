#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['armpi_fpv_kinematics'],
    package_dir={'': 'src'},
)

setup(**d)
