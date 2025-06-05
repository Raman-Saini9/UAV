# import os
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import ThisLaunchFileDir
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.actions import Node

# def generate_launch_description():
#     world_file = os.path.join(
#         get_package_share_directory('UAV_2'),
#         'launch',
#         'world.sdf'
#     )

#     return LaunchDescription([
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([
#                 os.path.join(
#                     get_package_share_directory('ros_gz_sim'),
#                     'launch',
#                     'gz_sim.launch.py'
#                 )
#             ]),
#             launch_arguments={'gz_args': world_file}.items()
#         ),
#         Node(
#             package='ros_gz_bridge',
#             executable='parameter_bridge',
#             name='clock_bridge',
#             arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
#             output='screen'
#         ),
#         # /tf
#         Node(
#             package='ros_gz_bridge',
#             executable='parameter_bridge',
#             name='tf_bridge',
#             arguments=['/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
#             output='screen'
#         ),
#         # /tf_static
#         Node(
#             package='ros_gz_bridge',
#             executable='parameter_bridge',
#             name='tf_static_bridge',
#             arguments=['/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
#             output='screen'
#         ),
#         # /cmd_vel
#         Node(
#             package='ros_gz_bridge',
#             executable='parameter_bridge',
#             name='cmd_vel_bridge',
#             arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
#             output='screen'
#         ),
#         # /odom
#         Node(
#             package='ros_gz_bridge',
#             executable='parameter_bridge',
#             name='odom_bridge',
#             arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
#             output='screen'
#         ),
#         # /imu/data
#         Node(
#             package='ros_gz_bridge',
#             executable='parameter_bridge',
#             name='imu_bridge',
#             arguments=['/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU'],
#             output='screen'
#         ),
#         # /camera/image_raw
#         Node(
#             package='ros_gz_bridge',
#             executable='parameter_bridge',
#             name='camera_bridge',
#             arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
#             output='screen'
#         ),
#         # /lidar/scan
#         Node(
#             package='ros_gz_bridge',
#             executable='parameter_bridge',
#             name='lidar_bridge',
#             arguments=['/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
#             output='screen'
#         ),
#     ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge  

def generate_launch_description():
    package_name = 'UAV_2'
    
    declare_bridge_name_cmd = DeclareLaunchArgument(
        'bridge_name', 
        default_value='ros_gz_bridge',
        description='Name of ros_gz_bridge node'
    )
    
    # CHANGE THIS: Set your default config file path
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', 
        default_value=PathJoinSubstitution([
            FindPackageShare(package_name),
            'config',
            'bridge.yaml'
        ]),
        description='YAML config file'
    )

    # CHANGE THIS: Set your default world file path
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            FindPackageShare(package_name),
            'launch', 
            'world.sdf'  # Change to your world filename
        ]),
        description='Path to the world.sdf file'
    )

    # CHANGE THIS: Modify the gazebo command based on your version
    # Option 1: For Gazebo Garden/Harmonic
    gazebo_launch = ExecuteProcess(
        cmd=['gz', 'sim', LaunchConfiguration('world_file'), '-v', '4'],
        output='screen'
    )

    # Create the bridge
    bridge = RosGzBridge(
        bridge_name=LaunchConfiguration('bridge_name'),
        config_file=LaunchConfiguration('config_file'),
    )

    # Create the launch description and populate
    ld = LaunchDescription([
        declare_bridge_name_cmd,
        declare_config_file_cmd,
        declare_world_file_cmd,
        gazebo_launch,
        bridge,
    ])

    return ld