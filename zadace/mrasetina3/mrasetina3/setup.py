from setuptools import setup

package_name = 'mrasetina3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matia',
    maintainer_email='mrasetina@riteh.hr',
    description='Mobilna robotika DZ3',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator=mrasetina3.navigator:main'
        ],
    },
)
