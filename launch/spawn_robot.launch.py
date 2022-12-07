import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    launch_file_dir_local = os.path.join(get_package_share_directory('ros2_roomba_obstacle_avoidance'), 'launch')
    # Get the urdf file
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )


    # urdf_path = os.path.join(
    #     get_package_share_directory('roomba_ros2'),
    #     'models',
    #     model_folder,
    #     'model.sdf'
    # )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    rosbag_flag = LaunchConfiguration('rosbag_flag', default=False)

    # launch arguments
    x_position = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    y_position = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    rosbag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir_local, 'ros_bag.launch.py')
        )
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(x_position)
    ld.add_action(y_position)
    ld.add_action(rosbag)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld