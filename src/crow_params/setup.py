from setuptools import setup

package_name = 'crow_params'

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
    requires=["zmq"],
    zip_safe=True,
    maintainer='RichardHajek',
    maintainer_email='richard.m.hajek@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = crow_params.server_launcher:main'
        ],
    },
)
