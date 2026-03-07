from setuptools import find_packages, setup

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'weights']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Team',
    maintainer_email='marlambekovdaulet@gmail.com',
    description='Object Detection and Reporting Package for Australian Rover Challenge 2026',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_reporter=object_detection.object_reporter:main',
        ],
    },
)
