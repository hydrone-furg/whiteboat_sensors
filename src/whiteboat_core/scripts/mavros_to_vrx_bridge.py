#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCOut
from std_msgs.msg import Float64


# Limites físicos do WAM-V (calculado via formula do xacro)
THRUST_MAX = 5000.0   # N (máx por thruster)
PWM_NEUTRAL = 1500    # μs
PWM_RANGE   = 500     # μs (desvio máximo de cada lado)


def pwm_to_normalized(pwm: int) -> float:
    """Converte valor PWM (1000-2000 μs) para valor normalizado [-1.0, 1.0]."""
    return max(-1.0, min(1.0, (pwm - PWM_NEUTRAL) / PWM_RANGE))


class MavrosToVrxBridge(Node):
    def __init__(self):
        super().__init__('mavros_to_vrx_bridge')

        # Parâmetros configuráveis
        self.declare_parameter('thrust_max', THRUST_MAX)
        self.declare_parameter('mavros_namespace', 'whiteboat')
        self.declare_parameter('wamv_namespace', 'wamv')
        self.declare_parameter('steering_channel', 0)   # SERVO1 → Steering
        self.declare_parameter('throttle_channel', 2)   # SERVO3 → Throttle

        thrust_max        = self.get_parameter('thrust_max').value
        mavros_ns         = self.get_parameter('mavros_namespace').value
        wamv_ns           = self.get_parameter('wamv_namespace').value
        self.steer_ch     = self.get_parameter('steering_channel').value
        self.throttle_ch  = self.get_parameter('throttle_channel').value
        self.thrust_max   = thrust_max

        # Publishers para os thrusters do VRX
        self.left_pub  = self.create_publisher(
            Float64, f'{wamv_ns}/thrusters/left/thrust', 10)
        self.right_pub = self.create_publisher(
            Float64, f'{wamv_ns}/thrusters/right/thrust', 10)

        # Subscriber nos outputs RC do MAVROS
        self.create_subscription(
            RCOut,
            f'/{mavros_ns}/rc/out',
            self.rc_out_callback,
            10
        )

        self.get_logger().info(
            f'Ponte MAVROS→VRX iniciada | '
            f'steering_ch={self.steer_ch} throttle_ch={self.throttle_ch} | '
            f'thrust_max={self.thrust_max} N'
        )

    def rc_out_callback(self, msg: RCOut):
        channels = msg.channels
        if len(channels) <= max(self.steer_ch, self.throttle_ch):
            return

        throttle_pwm = channels[self.throttle_ch]
        steering_pwm = channels[self.steer_ch]

        # PWM 0 = canal inativo (SITL ainda não armado)
        if throttle_pwm == 0 and steering_pwm == 0:
            return

        throttle = pwm_to_normalized(throttle_pwm)  # [-1, 1]
        steering  = pwm_to_normalized(steering_pwm)  # [-1, 1] (neg=esq, pos=dir)

        # Mixagem diferencial: rover-style
        #   left  = throttle + steering
        #   right = throttle - steering
        left_norm  = max(-1.0, min(1.0, throttle + steering))
        right_norm = max(-1.0, min(1.0, throttle - steering))

        left_thrust  = Float64(data=left_norm  * self.thrust_max)
        right_thrust = Float64(data=right_norm * self.thrust_max)

        self.left_pub.publish(left_thrust)
        self.right_pub.publish(right_thrust)

        self.get_logger().debug(
            f'PWM steer={steering_pwm} throttle={throttle_pwm} → '
            f'L={left_thrust.data:.1f}N R={right_thrust.data:.1f}N'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MavrosToVrxBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
