from setuptools import setup

package_name = "crow_detector"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RichardHajek",
    maintainer_email="richard.m.hajek@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector = crow_detector.detector_node:main",
            "aruco_detector = crow_detector.aruco_detector_node:main",
            "drawer_processing = crow_detector.drawer_processing_node:main",
            "scene_setup_1 = crow_detector.aruco_detector_node:scene_experimental_setup_node"
        ],
    },
)
