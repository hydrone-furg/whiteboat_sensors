from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Inclui o launch principal (MAVROS + câmera)
    whiteboat_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whiteboat_core'),
                'launch',
                'whiteboat.launch.py'
            )
        )
    )

    # Gravação de bag com ros2 bag record (equivalente ao rosbag record do ROS 1)
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', os.path.join(
                get_package_share_directory('whiteboat_localization'),
                'bags',
                'whiteboat_mission'
            ),
            '/whiteboat/mavros/state',
            '/whiteboat/mavros/global_position/fix',
            '/whiteboat/mavros/imu/data',
            '/whiteboat/mavros/rc/in',
            '/whiteboat/tf',
            '/whiteboat/tf_static',
            '/whiteboat/diagnostics',
        ],
        output='screen',
    )

    return LaunchDescription([
        whiteboat_core_launch,
        rosbag_record,
    ])
