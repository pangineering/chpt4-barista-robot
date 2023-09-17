import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_context import LaunchContext
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

# This is the function launch system will look for
def generate_launch_description():
    # Data input
    urdf_file = 'barista_robot_model.urdf'
    package_description = "barista_robot_description"
    robot_dir = os.path.join(get_package_share_directory('barista_robot_description'), 'urdf')
    robot_file = os.path.join(robot_dir, 'barista_robot_model.urdf')

    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)
    pkg_ros2 = get_package_share_directory('barista_robot_description')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true', description='Set to "false" to run headless.'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    launch_file_dir = os.path.join(pkg_ros2, 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'gui': gui}.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'baristabot',
            '-x', '0',
            '-y', '0',
            '-z', '1',
            '-file', robot_file,
        ]
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'robot.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    # Create and return launch description object
    return LaunchDescription(
        [
            declare_gui,
            gazebo,
            spawn_entity,
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node
        ]
    )
