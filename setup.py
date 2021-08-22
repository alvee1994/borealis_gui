#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['borealis_gui'],
    package_dir={'': 'src'},
    scripts=['scripts/borealis_gui']
)

setup(**d)
