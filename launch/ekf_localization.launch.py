from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Correct full path to rosbag file in source directory
    rosbag_file = os.path.join(
        os.getenv('HOME'),
        'ros2_ws',
        'src',
        'motion_update_pkg',
        'rosbag_localisation',
        'rosbag2_2025_04_06-23_11_57_0.db3'
    )

    return LaunchDescription([
        # Launch EKF Localization Node - NO REMAPPING
        Node(
            package='motion_update_pkg',
            executable='ekf_localization_node',
            name='ekf_localization_node',
            output='screen'
        ),
        
        # Launch EKF Visualizer Node
        Node(
            package='motion_update_pkg',
            executable='ekf_visualizer_node',
            name='ekf_visualizer_node',
            output='screen'
        ),
        
        # Play the rosbag file (in loop for debugging)
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', rosbag_file, '--loop'],
            output='screen'
        )
    ])
