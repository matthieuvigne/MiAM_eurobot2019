#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import setup


setup(
    name='miam_py',
    version='0.1.0',
    description='MiAM robotics log processing',
    long_description=open('README.md').read(),
    packages=['miam_py'],
    package_dir={'': 'src'},
    scripts=["scripts/miam_plot"],
    zip_safe=False)
