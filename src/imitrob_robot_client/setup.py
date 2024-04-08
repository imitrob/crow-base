from setuptools import setup

package_name = 'imitrob_robot_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Radoslav Skoviera',
    maintainer_email='radoslav.skoviera@cvut.cz',
    description='Client package for the imitrob robot server',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_client_async = imitrob_robot_client.test_client_async:main',
            'test_client_sync = imitrob_robot_client.test_client_sync:main'
        ],
    },
)
