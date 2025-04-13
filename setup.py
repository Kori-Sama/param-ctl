# -*- coding: utf-8 -*-

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="param-controller",
    version="0.1.6",
    author="Kori-Sama",
    author_email="Miyohashikori457@gmail.com",
    description="A lightweight parameter remote controller for embedded device debugging",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Kori-Sama/param-ctl",
    packages=find_packages(exclude=["param_ctl.templates.*", "param_ctl.static.*"]),
    include_package_data=True,
    package_data={
        "param_ctl": ["templates/*.html", "static/*.js", "static/*.css"],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)
