from setuptools import find_packages, setup

package_name = 'gizmo_scripts'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CoreBotics-Lab',
    maintainer_email='hayisyed@gmail.com',
    description='Application layer Python missions and autonomous navigation scripts for Gizmo robot using Nav2 Simple Commander API',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'go_to_goal_slam_toolbox_nav2 = gizmo_scripts.go_to_goal_slam_toolbox_nav2:main',
            'go_to_goal_amcl_nav2 = gizmo_scripts.go_to_goal_amcl_nav2:main',

            # Practice and Learning Nav2 Simple Commander API
            'practice_nav2Api = gizmo_scripts.practice.practice_nav2_api:main',
        ],
    },
)
