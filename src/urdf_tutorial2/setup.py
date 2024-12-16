from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'urdf_tutorial2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jw7720',
    maintainer_email='jiin772@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'projectile_spawner = urdf_tutorial2.projectile_spawner:main',
            'projectile_spawner2 = urdf_tutorial2.projectile_spawner2:main',
        ],
    },
)