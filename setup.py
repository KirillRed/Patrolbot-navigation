from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'patrolbot_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kyrylo Redenskyi',
    maintainer_email='kredensk@andrew.cmu.edu',
    description='PatrolBot Navigation Logic',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator = patrolbot_nav.navigator:main',
        ],
    },
)