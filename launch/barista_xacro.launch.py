import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Package description and path
    package_description = "barista_robot_description"
    robot_model_path = os.path.join(get_package_share_directory(package_description))
    xacro_file = os.path.join(robot_model_path, 'xacro', 'barista.urdf.xacro')

    # Process Xacro file into URDF
    doc = xacro.process_file(xacro_file)
    params = {'robot_description': doc.toxml()}

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={"verbose": "false", 'pause': 'true'}.items(),
    )

    # GUI argument
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true', description='Set to "false" to run headless.'
    )

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'barista', '-x', '1.0', '-y', '1.0', '-z', '0.2',
                   '-topic', 'robot_description'],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_config_dir = os.path.join(robot_model_path, 'rviz', 'robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        declare_gui,
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        joint_state_publisher_node,
        rviz_node
    ])
