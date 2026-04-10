from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Define se roda o nó de câmera simulada (VRX) ou câmera real'
    )

    sim = LaunchConfiguration('sim')

    # Nó da câmera real do Whiteboat
    camera_real_node = Node(
        package='whiteboat_cam',
        executable='whiteboat_cam.py',
        name='camera_driver_node',
        output='screen',
        parameters=[{
            'camera_index': 0,
            'wait_for_mavros': False,
        }],
        condition=UnlessCondition(sim),
    )

    # Nó da câmera simulada integrada com VRX
    camera_sim_node = Node(
        package='whiteboat_cam',
        executable='camera_sim_node.py',
        name='camera_sim_node',
        output='screen',
        condition=IfCondition(sim),
    )

    return LaunchDescription([
        sim_arg,
        camera_real_node,
        camera_sim_node,
    ])
