from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Define se roda o nó de câmera simulada (VRX) ou câmera real'
    )
    frame_width_arg = DeclareLaunchArgument('frame_width', default_value='320')
    frame_height_arg = DeclareLaunchArgument('frame_height', default_value='240')
    fps_arg = DeclareLaunchArgument('fps', default_value='10.0')

    sim = LaunchConfiguration('sim')
    frame_width = LaunchConfiguration('frame_width')
    frame_height = LaunchConfiguration('frame_height')
    fps = LaunchConfiguration('fps')

    # Nó da câmera real do Whiteboat
    camera_real_node = Node(
        package='whiteboat_cam',
        executable='whiteboat_cam.py',
        name='camera_driver_node',
        output='screen',
        parameters=[{
            'camera_index': 0,
            'wait_for_mavros': False,
            'frame_width': frame_width,
            'frame_height': frame_height,
            'fps': fps,
            'publish_only_when_subscribed': True,
            'enable_preview': False,
        }],
        condition=UnlessCondition(sim),
    )

    # Nó da câmera simulada integrada com VRX
    camera_sim_node = Node(
        package='whiteboat_cam',
        executable='camera_sim_node.py',
        name='camera_sim_node',
        output='screen',
        parameters=[{
            'passthrough': True,
            'draw_overlay': False,
            'publish_only_when_subscribed': True,
            'max_fps': fps,
        }],
        condition=IfCondition(sim),
    )

    return LaunchDescription([
        sim_arg,
        frame_width_arg,
        frame_height_arg,
        fps_arg,
        camera_real_node,
        camera_sim_node,
    ])
