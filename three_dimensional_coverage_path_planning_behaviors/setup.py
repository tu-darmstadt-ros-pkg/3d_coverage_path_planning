#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages = ['three_dimensional_coverage_path_planning_behaviors'],
    package_dir = {'': 'src'}
)

setup(**d)
