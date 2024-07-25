import launch
import launch_ros
import os
import xacro

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='armv1').find('armv1')
    print(pkg_share)
    xacro_file = os.path.join(pkg_share, 'urdf', 'arm_exten.xacro')
    
    # Generate URDF from Xacro
    doc = xacro.process_file(xacro_file)
    urdf_content = doc.toprettyxml(indent='  ')
    
    # Save the generated URDF to a temporary file
    urdf_temp_path = '/tmp/new.urdf'
    with open(urdf_temp_path, 'w') as f:
        f.write(urdf_content)

    robot_description = {'robot_description': urdf_content}

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(launch.substitutions.LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                             description='Flag to enable joint_state_publisher_gui'),
        robot_state_publisher_node,
        rviz_node
    ])

if __name__ == '__main__':
    generate_launch_description()
