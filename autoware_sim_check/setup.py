from setuptools import setup

package_name = 'autoware_sim_check'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vehicle_check.launch.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='NEWSLab NTU',
    maintainer_email='jerry73204@gmail.com',
    description="vehicle_check: a simulated car's vehicle side against its vehicle spec",
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'vehicle_check = autoware_sim_check.vehicle_check:main',
    ]},
)
