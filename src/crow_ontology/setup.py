from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'crow_ontology'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'resource']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'data'), glob('data/*.*'))
    ],
    install_requires=['setuptools'],
    requires=["knowl"],
    zip_safe=True,
    maintainer='syxtreme',
    maintainer_email='radoslav.skoviera@cvut.cz',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'server = crow_ontology.crowracle_server:main_ros',
            'adder = crow_ontology.add_to_database:main',

            'ovis = crow_ontology.user_interface.onto_vision:main',
            'oterm = crow_ontology.user_interface.onto_terminal:main',
            'oren = crow_ontology.user_interface.onto_render:main',

            'tester = crow_ontology.examples.tester:main',
            'tester2 = crow_ontology.examples.tester2:main',
            'tester3 = crow_ontology.examples.tester3:main',
            'tester4 = crow_ontology.examples.tester4:main',
            'query_bench = crow_ontology.examples.query_bench:main',
        ],
    },
)
