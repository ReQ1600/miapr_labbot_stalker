from setuptools import setup

package_name = 'person_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@student.pl',
    description='ROS2 person detector',
    license='MIT',
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    entry_points={
        'console_scripts': [
            'person_follower = person_detector.person_detector_node:main'
        ],
    },
)