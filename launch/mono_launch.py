import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'ros2_orb_slam3'
    
    # Declare launch arguments
    settings_name_arg = DeclareLaunchArgument(
        'settings_name', default_value='EuRoC',
        description='Name of the .yaml file containing camera intrinsics'
    )
    image_seq_arg = DeclareLaunchArgument(
        'image_seq', default_value='V2_01_easy',
        description='Name of the image sequence to run'
    )
    visual_scale_arg = DeclareLaunchArgument(
        'visual_scale', default_value='1.0',
        description='Visual scale factor for monocular SLAM'
    )

    
    # Monocular SLAM C++ Node
    # Get current environment and add/override necessary variables
    env = dict(os.environ)
    env['LD_LIBRARY_PATH'] = env.get('LD_LIBRARY_PATH', '') + ':/home/viswa/task_01_visual_odometry/Pangolin/build'
    if 'HOME' not in env:
        env['HOME'] = '/home/viswa'
    env['ROS_HOME'] = os.path.join(env['HOME'], '.ros')

    mono_node = Node(
        package=pkg_name,
        executable='mono_node_cpp',
        name='mono_slam_cpp',
        output='screen',
        parameters=[{
            'node_name_arg': 'mono_slam_cpp',
            'map_frame_id': 'map',
            'camera_link_frame_id': 'camera_link',
            'visual_scale': LaunchConfiguration('visual_scale')
        }],
        env=env
    )
    
    # Python Driver Node
    driver_node = Node(
        package=pkg_name,
        executable='mono_driver_node.py',
        name='mono_driver',
        output='screen',
        parameters=[{
            'settings_name': LaunchConfiguration('settings_name'),
            'image_seq': LaunchConfiguration('image_seq')
        }]
    )

    # RViz Node
    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'mono_view.rviz'
    )
    # Default to local path if not installed yet (for first run)
    if not os.path.exists(rviz_config_path):
        rviz_config_path = '/home/viswa/task_01_visual_odometry/rviz/mono_view.rviz'

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        settings_name_arg,
        image_seq_arg,
        visual_scale_arg,
        mono_node,
        driver_node,
        rviz_node
    ])
