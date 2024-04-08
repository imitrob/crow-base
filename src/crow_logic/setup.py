from setuptools import setup, find_packages

package_name = 'crow_logic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['resource']),
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
            'logic = crow_logic.logic:main',
            'logic404 = crow_logic.logic_404:main',
            'dummy = crow_logic.testing.dummy_action_robot:main',
            'dummy_nl_input = crow_logic.testing.dummy_nl_input_curses:main',
            'ptest = crow_logic.testing.test_param:main',
            'stest = crow_logic.testing.test_param_server:main',
            'aplanner = crow_control.assembly_planner:main',
            'famonitor = crow_control.assembly_monitor_fake:main',
        ],
    },
)
