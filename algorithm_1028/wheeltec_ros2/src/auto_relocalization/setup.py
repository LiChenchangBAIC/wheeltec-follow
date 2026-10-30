from setuptools import find_packages, setup

package_name = 'auto_relocalization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/auto_relocalization.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'pyyaml',
        'pillow',
        'numpy'
    ],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_relocalization = auto_relocalization.simple_relocalization:main',
        ],
    },
)
