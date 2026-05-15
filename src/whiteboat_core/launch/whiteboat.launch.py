from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Define se roda em modo simulação (VRX) ou hardware real'
    )
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Inicializa câmera. Pode ser false em UAV para reduzir CPU.'
    )
    camera_fps_arg = DeclareLaunchArgument('camera_fps', default_value='10.0')
    mavros_param_plugin_arg = DeclareLaunchArgument(
        'mavros_param_plugin',
        default_value='false',
        description='Carrega o plugin param do MAVROS. Desativado por padrão para evitar spam e reduzir CPU.'
    )
    mavros_param_log_level_arg = DeclareLaunchArgument(
        'mavros_param_log_level',
        default_value='warn',
        description='Nível de log do logger whiteboat.param do MAVROS.'
    )

    sim = LaunchConfiguration('sim')
    enable_camera = LaunchConfiguration('enable_camera')
    camera_fps = LaunchConfiguration('camera_fps')
    mavros_param_plugin = LaunchConfiguration('mavros_param_plugin')
    mavros_param_log_level = LaunchConfiguration('mavros_param_log_level')

    fcu_url_expr = PythonExpression([
        "'udp://127.0.0.1:14550@' if '", sim, "' == 'true' else '/dev/ttyACM0:57600'"
    ])

    mavros_common_params = {
        'use_sim_time': PythonExpression(["'true' if '", sim, "' == 'true' else 'false'"]),
        'fcu_url': fcu_url_expr,
        'target_system_id': 1,
        'system_id': 255,
        'component_id': 240,
    }
    mavros_denied_plugins = [
        'actuator_control',
        'adsb',
        'altitude',
        'cam_imu_sync',
        'camera',
        'cellular_status',
        'companion_process_status',
        'debug_value',
        'distance_sensor',
        'esc_status',
        'esc_telemetry',
        'fake_gps',
        'ftp',
        'geofence',
        'gimbal_control',
        'gps_input',
        'gps_rtk',
        'gps_status',
        'guided_target',
        'hil',
        'home_position',
        'landing_target',
        'log_transfer',
        'mag_calibration_status',
        'manual_control',
        'mocap_pose_estimate',
        'mount_control',
        'nav_controller_output',
        'obstacle_distance',
        'obstacle_distance_3d',
        'odometry',
        'onboard_computer_status',
        'open_drone_id',
        'optical_flow',
        'play_tune',
        'px4flow',
        'rallypoint',
        'rangefinder',
        'setpoint_accel',
        'setpoint_attitude',
        'setpoint_position',
        'setpoint_raw',
        'setpoint_trajectory',
        'setpoint_velocity',
        'sim_state',
        'tdr_radio',
        'terrain',
        'trajectory',
        'tunnel',
        'vfr_hud',
        'vibration',
        'vision_pose',
        'vision_speed',
        'waypoint',
        'wheel_odometry',
        'wind_estimation',
    ]
    mavros_common_args = ['--ros-args', '--log-level', ['whiteboat.param:=', mavros_param_log_level]]

    # Nó do MAVROS sem plugin param: default leve para sim/hardware embarcado.
    mavros_node_light = Node(
        package='mavros',
        executable='mavros_node',
        namespace='whiteboat',
        output='screen',
        parameters=[{
            **mavros_common_params,
            'plugin_denylist': [*mavros_denied_plugins, 'param'],
        }],
        arguments=mavros_common_args,
        condition=UnlessCondition(mavros_param_plugin),
    )

    # Nó do MAVROS com plugin param, caso seja necessário ler/escrever parâmetros da FCU.
    mavros_node_with_param = Node(
        package='mavros',
        executable='mavros_node',
        namespace='whiteboat',
        output='screen',
        parameters=[{
            **mavros_common_params,
            'plugin_denylist': mavros_denied_plugins,
        }],
        arguments=mavros_common_args,
        condition=IfCondition(mavros_param_plugin),
    )

    # Inclui o launch de câmera
    whiteboat_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whiteboat_cam'),
                'launch',
                'whiteboat_cam.launch.py'
            )
        ),
        launch_arguments={'sim': sim, 'fps': camera_fps}.items(),
        condition=IfCondition(enable_camera),
    )

    # Nó Ponte MAVROS -> VRX (roda apenas no modo simulação)
    bridge_node = Node(
        package='whiteboat_core',
        executable='mavros_to_vrx_bridge.py',
        name='mavros_to_vrx_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(sim)
    )

    return LaunchDescription([
        sim_arg,
        enable_camera_arg,
        camera_fps_arg,
        mavros_param_plugin_arg,
        mavros_param_log_level_arg,
        mavros_node_light,
        mavros_node_with_param,
        whiteboat_cam_launch,
        bridge_node,
    ])
