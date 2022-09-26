from setuptools import setup
import os
from glob import glob
package_name = 'tello_mod'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        #(os.path.join('share', package_name, ), glob('config/*.yaml')),    

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hongli',
    maintainer_email='hongli@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tello_teleop = tello_mod.tello_teleop:main',
            'image_receive = tello_mod.image_receive:main',
            'image_receive_new = tello_mod.image_receive_new:main',
            'base_tf2_broadcaster =tello_mod.base_tf2_broadcaster:main',
            'tello_tf_listener = tello_mod.tello_tf_listener:main',
            'tf_listener_example = tello_mod.tf_listener_example:main',
            'hsv_filter = tello_mod.hsv_filter:main',
            'fake_camera = tello_mod.fake_camera:main',
            'camera_receive_ros2 = tello_mod.camera_receive_ros2:main',
            'tello_tf = tello_mod.tello_tf:main',
            'graph = tello_mod.graph:main',
            'graph2 = tello_mod.graph2:main',
            'pid_pub = tello_mod.pid_pub:main',
            'pid_teleop = tello_mod.pid_teleop:main',
            'strategy = tello_mod.strategy:main',
            'blind_fly_server = tello_mod.blind_fly_server:main',
            'slow_rotate_server = tello_mod.slow_rotate_server:main',
            'slow_forward_server = tello_mod.slow_forward_server:main',
            'ORB_pid_pub2 = tello_mod.ORB_pid_pub2:main',
            'ORB_pid_teleop = tello_mod.ORB_pid_teleop:main',
            'orb_pid_teleop_sim = tello_mod.orb_pid_teleop_sim:main',
            'polynomial_path_planner = tello_mod.polynomial_path_planner:main',
            'orb_listener = tello_mod.orb_listener:main',
            'gate_waypoint_pub = tello_mod.gate_waypoint_pub:main',
            'sub = tello_mod.sub:main',


        ],
    },
)
