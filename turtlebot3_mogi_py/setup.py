from setuptools import find_packages, setup
from glob import glob

package_name = 'turtlebot3_mogi_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/network_model', glob('network_model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Dudas',
    maintainer_email='david.dudas@outlook.com',
    description='BME MOGI ROS course related python nodes to the official Turtlebot 3 packages',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower = turtlebot3_mogi_py.line_follower:main',
            'save_training_images = turtlebot3_mogi_py.save_training_images:main',
            'line_follower_cnn = turtlebot3_mogi_py.line_follower_cnn:main',
            'line_follower_cnn_robot = turtlebot3_mogi_py.line_follower_cnn_robot:main',
        ],
    },
)
