import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    description_package_name = "barista_robot_description"

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Define the launch arguments for the Gazebo launch file
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock instead of system clock'
    )
    # Remove either the default value or the assigned value for the robot_name property
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        # Remove the default value or the assigned value, but not both.
        default_value='my_robot',
        description='Name of the robot'
    )
    # Include the Gazebo launch file with the modified launch arguments
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'verbose': 'false', 'pause': 'true', 'use_sim_time': use_sim_time}.items(),
    )

    # Define the robot model files to be used
    robot_desc_file = "barista_2.urdf.xacro"
    robot_desc_path = os.path.join(get_package_share_directory(
        "barista_robot_description"), "xacro", robot_desc_file)
    robot_model_path = os.path.join(get_package_share_directory('barista_robot_description'))
    xacro_file = os.path.join(robot_model_path, 'xacro', 'barista_2.urdf.xacro')
    robot_name_1 = "morty"
    robot_name_2 = "rick"

    # Convert Xacro file into URDF
    doc1 = xacro.parse(open(xacro_file))
    xacro.process_doc(doc1,mappings={'robot_name' : robot_name_1})
    params1 = {'robot_description': doc1.toxml()}

    doc2 = xacro.parse(open(xacro_file))
    xacro.process_doc(doc2,mappings={'robot_name' : robot_name_2})
    params2 = {'robot_description': doc2.toxml()}



    rsp_robot11 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name_1,
        parameters=[{'frame_prefix': robot_name_1 + '/'},params1],
        output="screen",
        remappings=[('/scan', robot_name_1 + '/scan')]
    )

    # Robot 2 - Robot State Publisher
    rsp_robot21 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher2',
        namespace=robot_name_2,
        parameters=[{'frame_prefix': robot_name_2 + '/'},params2],
        output="screen",
        remappings=[('/scan', robot_name_2 + '/scan')]
    )



    # Joint State Publisher GUI for Robot 1
    jspn_robot1 = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui1',
        namespace=robot_name_1,  # Correct namespace
        output='screen'
    )

    # Joint State Publisher GUI for Robot 2
    jspn_robot2 = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui2',
        output='screen',
        namespace=robot_name_2
    )

    # Spawn Robot 1 in Gazebo
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot1', '-x', '0.0', '-y', '0.0', '-z', '0.0',
                   '-topic', robot_name_1+'/robot_description'],
    )

    # Spawn Robot 2 in Gazebo
    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot2', '-x', '1.0', '-y', '1.0', '-z', '0.0',
                   '-topic', robot_name_2+'/robot_description']
    )
    # TF2 Node for Robot 1
    tf_node_robot1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_publisher_robot1',
        namespace=robot_name_1,
        output='screen',
        arguments=[
            '0', '0', '0', '0', '0', '0', 'world', f'{robot_name_1}/base_link'
        ]
    )

    # TF2 Node for Robot 2
    tf_node_robot2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_publisher_robot2',
        namespace=robot_name_2,
        output='screen',
        arguments=[
            '1.0', '1.0', '0', '0', '0', '0', 'world', f'{robot_name_2}/base_link'
        ]
    )

    rviz_config_dir = os.path.join(get_package_share_directory(description_package_name), 'rviz', 'robot2.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )
    return LaunchDescription([
        declare_use_sim_time,
        rviz_node,
        gazebo,
        #declare_robot_name,
        rsp_robot11,
        rsp_robot21,
        jspn_robot1,
        jspn_robot2,
        tf_node_robot1,  # Add TF node for Robot 1
        tf_node_robot2,  # Add TF node for Robot 2
        spawn_robot1,
        spawn_robot2
    ])
