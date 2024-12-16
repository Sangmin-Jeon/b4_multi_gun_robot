from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'urdf_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jw7720',
    maintainer_email='jw7720@example.com',
    description='URDF tutorial package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'projectile_spawner = urdf_tutorial.projectile_spawner:main',
        ],
    },
)
