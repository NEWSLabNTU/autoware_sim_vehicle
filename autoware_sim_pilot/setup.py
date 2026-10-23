from setuptools import setup

package_name = 'autoware_sim_pilot'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/example_poses.yaml']),
        ('share/' + package_name + '/config/poses', ['config/poses/Town01.yaml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Drive Autoware to a goal over its API, and capture poses from RViz: any simulated car',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'auto_drive = autoware_sim_pilot.auto_drive:main',
            'capture_poses = autoware_sim_pilot.capture_poses:main',
        ],
    },
)
