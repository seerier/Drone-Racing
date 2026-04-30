from setuptools import setup
from glob import glob
import os

package_name = 'crazyradio_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'rotorpy'],
    zip_safe=True,
    maintainer='Lorenzo Bianchi',
    maintainer_email='lnz.bnc@gmail.com',
    description='Crazyradio driver node.',
    license='GNU GPL v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyradio_driver = crazyradio_driver.crazyradio_driver_app:main',
        ],
    },
)
