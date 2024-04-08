import os
from glob import glob

from setuptools import setup

package_name = "cam_calib"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="richard-hajek",
    maintainer_email="richard.m.hajek@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            'calibrator = cam_calib.calibrator:main',
        ],
    },
)
