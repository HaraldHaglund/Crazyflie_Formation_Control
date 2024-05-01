from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'frtn70'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('lib', package_name), glob(os.path.join(package_name, 'FrameListener.py'))),
        (os.path.join('lib', package_name), glob(os.path.join(package_name, 'GraphicsHandler.py'))),
        (os.path.join('lib', package_name), glob(os.path.join(package_name, 'MultiPathGraphicsHandler.py'))),
        (os.path.join('lib', package_name), glob(os.path.join(package_name, 'Crazyflie.py'))),
        (os.path.join('lib', package_name), glob(os.path.join(package_name, 'Astar.py'))),
        (os.path.join('lib', package_name), glob(os.path.join(package_name, 'singlepathswc.py'))),
        (os.path.join('lib', package_name), glob(os.path.join(package_name, 'single.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='badodds',
    maintainer_email='01ste02@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swc = frtn70.swc:main',
            'pathswc = frtn70.pathswc:main',
            'singlepathswc = frtn70.singlepathswc:main',
            'simple = frtn70.single:main'
        ],
    },
)
