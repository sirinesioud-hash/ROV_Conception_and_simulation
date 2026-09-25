import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Static Transform Publisher for Sonar
    tf_sonar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_sonar',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', 
            '--roll', '0', '--pitch', '0', '--yaw', '0', 
            '--frame-id', 'world', 
            '--child-frame-id', 'bluerov2/sonar_link/sonar_sensor'
        ],
        output='screen'
    )

    # 2. Static Transform Publisher for Camera
    tf_camera_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', 
            '--roll', '0', '--pitch', '0', '--yaw', '0', 
            '--frame-id', 'world', 
            '--child-frame-id', 'bluerov2/camera_link/camera'
        ],
        output='screen'
    )

    # 3. Gazebo Sim (gz sim)
    gazebo_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '3', '-r', 'bluerov2_underwater.world'],
        output='screen'
    )

    # 4. ROS-Gazebo Parameter Bridge
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{
            'config_file': '/home/bluerov2/ros2_ws/bluerov2_bridge.yaml',
            'use_sim_time': True
        }],
        output='screen'
    )

    # 5. RViz2 with custom config file
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/home/bluerov2/ros2_ws/camera_sonar_view.rviz'],
        parameters=[{'use_sim_time': True}], # Enabling use_sim_time is recommended when using Gazebo
        output='screen'
    )

    info_node = Node(
        package='inspection_package',
        executable='inspection_node',
        name='inspection_node',
        output='screen'
    )

    keyboard_controller = Node(
        package='rov_thruster',
        executable='keyboard_controller_node',
        name='keyboard_controller_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo_sim,
        tf_sonar_node,
        tf_camera_node,
        ros_gz_bridge_node,
        rviz2_node,
        info_node,
        keyboard_controller 

    ])