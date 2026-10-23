from setuptools import find_packages, setup

package_name = 'wheeltec_ultrasonic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ultrasonic_pose.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
         'ultrasonic_enum_node = wheeltec_ultrasonic.ultrasonic_enum:main',
         'ultrasonic_points_node = wheeltec_ultrasonic.ultrasonic_points:main',
         'rear_ultrasonic_node = wheeltec_ultrasonic.rear_ultrasonic:main',

        ],
    },
)
