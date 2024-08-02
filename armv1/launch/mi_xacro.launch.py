import launch
import launch_ros
import os
import xacro

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='armv1').find('armv1')
    xacro_path = os.path.join(pkg_share, 'urdf', 'arm_mi.xacro')

    doc = xacro.process_file(xacro_path)
    urdf_content = doc.toprettyxml(indent='  ')

    # Save the generated URDF to a temporary file
    urdf_temp_path = '/tmp/new.urdf'
    with open(urdf_temp_path, 'w') as f:
        f.write(urdf_content)
    
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', xacro_path]), value_type=str
    )
    
    return launch.LaunchDescription([

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
            output='screen',
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
        ),

        launch_ros.actions.Node(
            package='controller_manager',
            executable='spawner',
            name = 'joint_traj_spawner',
            arguments=['joint_trajectory_controller'],
        ),

        launch_ros.actions.Node(
            package='controller_manager',
            executable='spawner',
            name = 'joint_state_spawner',
            arguments=['joint_state_broadcaster'],
        ),
        
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name = 'rviz2',
            output = 'screen',
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()