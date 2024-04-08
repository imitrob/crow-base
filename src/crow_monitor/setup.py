from setuptools import setup

package_name = 'crow_monitor'

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
    maintainer='RichardHajek',
    maintainer_email='richard.m.hajek@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'profiler = crow_monitor.profiler.profiler_node:main',
            'monitor = crow_monitor.monitor:main',
            'wxvis = crow_monitor.wx_visualizator:main',
            'images = crow_monitor.observations.image_viewer:main',
            'topics = crow_monitor.observations.topic_viewer:main',
            'timestamps = crow_monitor.observations.timestamp_watcher:main',
            'node_alive = crow_monitor.observations.node_alive_test:main',
            'check_alive = crow_monitor.observations.check:entrypoint',
        ],
    },
)
