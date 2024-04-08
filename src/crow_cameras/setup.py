import os
from glob import glob

from setuptools import setup, find_packages

package_name = "crow_cameras"

setup(
    name=package_name,
    version="0.0.0",
    # packages=[package_name],
    packages=find_packages(exclude=['test', 'resource', 'launch']),
    data_files=[
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/resource", glob("resource/*")),
        (f"share/ament_index/resource_index/packages", ["resource/" + package_name]),
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
            'calibrator = crow_cameras.calibrator:main',
        ],
    },
)
