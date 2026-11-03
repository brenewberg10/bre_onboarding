from setuptools import find_packages, setup

package_name = 'phase_3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pynput'],
    zip_safe=True,
    maintainer='bre',
    maintainer_email='breannanewberg@gmail.com',
    description='ROS 2 keyboard teleoperation for the PAIR vehicle',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop = phase_3.teleop:main',
        ],
    },
)
