from setuptools import find_packages, setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Team',
    maintainer_email='marlambekovdaulet@gmail.com',
    description='Autonomous Navigation Stack for Australian Rover Challenge 2026',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_manager=navigation.navigation_manager:main',
            'waypoint_handler=navigation.waypoint_handler:main',
            'frontier_explorer=navigation.frontier_exploration:main',
            'collision_recovery=navigation.collision_recovery:main',
            'autonomous_phase_manager=navigation.autonomous_phase_manager:main',
            'start_frame_reference=navigation.start_frame_reference:main',
        ],
    },
)
