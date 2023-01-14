from setuptools import setup
import os
from glob import glob

package_name = 'mrasetina1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matia',
    maintainer_email='mrasetina@riteh.hr',
    description='Mobilna robotika',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'odom_sub = mrasetina1.odom_subscriber:main',
            'nav = mrasetina1.nav:main',
            'goal_pose_subscriber = mrasetina1.goal_pose_listener:main'
        ],
    },
)
