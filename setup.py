from setuptools import find_packages, setup

package_name = 'path_smoothing_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/path_smoothing_control']),
        ('share/path_smoothing_control', ['package.xml']),
        ('share/path_smoothing_control/launch', ['launch/simulation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdulrahman',
    maintainer_email='abdulrahmantrm@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
    'main_node = path_smoothing_control.main_node:main',
    'rviz_marker_publisher = path_smoothing_control.rviz_marker_publisher:main',
],
},
)
