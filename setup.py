#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_interactive_image_view'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_interactive_image_view']
)

setup(**d)
