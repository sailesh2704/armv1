import launch
import launch_ros
import os
import xacro
import time

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='armv1').find('armv1')
    xacro_path = os.path.join(pkg_share, 'urdf', 'arm_gz.xacro')

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', xacro_path]), value_type=str
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gazebo_node = launch_ros.actions.Node(
        executable='/usr/bin/gazebo',
        arguments=['-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )

    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'arm',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_node,
        spawn_entity_node
    ])

if __name__ == '__main__':
    generate_launch_description()