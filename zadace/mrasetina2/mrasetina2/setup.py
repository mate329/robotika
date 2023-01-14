import os
from glob import glob
from setuptools import setup

package_name = 'mrasetina2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mrasetina',
    maintainer_email='mrasetina@riteh.hr',
    description='DZ2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goto_closest = mrasetina2.goto_closest:main'
        ],
    },
)
