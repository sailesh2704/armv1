import launch
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='armv1').find('armv1')
    xacro_path = os.path.join(pkg_share, 'urdf', 'arm_gz.xacro')

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', xacro_path]), value_type=str
    )

    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            name='gui', 
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'
        ),

        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=launch.conditions.UnlessCondition(
                launch.substitutions.LaunchConfiguration('gui')
            )
        ),

        launch_ros.actions.Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('gui')
            )
        ),

        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            cwd=os.path.expanduser('~'),
            shell=True
        ),

        launch_ros.actions.Node(
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

    ])

if __name__ == '__main__':
    generate_launch_description()
