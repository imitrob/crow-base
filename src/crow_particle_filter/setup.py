from setuptools import setup

package_name = 'crow_particle_filter'

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
            'locator = crow_particle_filter.locator:main',
            'filter = crow_particle_filter.filter_node:main',
            'pcl_cacher = crow_particle_filter.pcl_cacher:main',
        ],
    },
)
