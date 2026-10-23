from setuptools import setup

package_name = 'yolo_same_follow'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    package_data={package_name: ['bytetrack.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo_same_follow.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@local',
    description='Click-to-lock YOLO segmentation follower',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_same_follow = yolo_same_follow.follow_node:main',
            'export_seg_engine = yolo_same_follow.export_engine:main',
        ],
    },
)
