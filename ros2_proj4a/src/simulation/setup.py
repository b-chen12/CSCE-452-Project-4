from setuptools import setup
import glob
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name), glob('urdf/*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='osboxes',
    maintainer_email='osboxes@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'proj4a = simulation.proj4a:main',
            'nav_controller = simulation.nav_controller:main',
            'vel_translator = simulation.vel_translator:main',
        ],
    },
)
