from setuptools import find_packages, setup

package_name = 'phase_4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bre',
    maintainer_email='breannanewberg@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'localization_node = phase_4.localization_node:main',
            'planner_node = phase_4.planner_node:main',
            'vehicle_input_adapter = phase_4.vehicle_input_adapter:main',
        ],
    },
)
