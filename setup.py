#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import setup, find_packages

setup(
        name="general_msgs",
        author="hausen6",
        packages=find_packages("src"),
        package_dir={"": "src"},
        )
