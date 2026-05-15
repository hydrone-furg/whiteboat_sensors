#!/usr/bin/env python3
"""Bridge MAVROS → VRX via Gazebo Transport direto.

Recebe comandos RC Override / RC Out do MAVROS e publica thrust
diretamente no Gazebo Transport (bypassing o ros_gz_bridge que tem
incompatibilidade de tipos de mensagem ignition.msgs vs gz.msgs).
"""
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn, RCOut
import subprocess
import os
import signal


# Limites físicos do WAM-V (calculado via formula do xacro)
# max_thrust_cmd = ((x_u + x_uu * v_max) * v_max) / 2
# x_u=51.3, x_uu=72.4, v_max=7.71667 → 2353.53 N
THRUST_MAX = 2353.5   # N (máx por thruster, conforme model.urdf)
PWM_NEUTRAL = 1500    # μs
PWM_RANGE   = 500     # μs (desvio máximo de cada lado)

# Tópicos Gazebo Transport dos thrusters do WAM-V
GZ_LEFT_THRUST  = '/wamv/thrusters/left/thrust'
GZ_RIGHT_THRUST = '/wamv/thrusters/right/thrust'
GZ_MSG_TYPE     = 'gz.msgs.Double'


def normalize_namespace(namespace: str) -> str:
    """Return a namespace without surrounding slashes for predictable topic joins."""
    return str(namespace).strip('/')


def pwm_to_normalized(pwm: int) -> float:
    """Converte valor PWM (1000-2000 μs) para valor normalizado [-1.0, 1.0]."""
    return max(-1.0, min(1.0, (pwm - PWM_NEUTRAL) / PWM_RANGE))


class GzPublisher:
    """Publica valores no Gazebo Transport via subprocess (bypass bridge ROS)."""

    def __init__(self, gz_topic: str):
        self.topic = gz_topic
        self._last_value = None
        self._devnull = open(os.devnull, 'w')

    def publish(self, value: float):
        """Publica apenas se o valor mudar significativamente (> 1N)."""
        if self._last_value is not None and abs(value - self._last_value) < 1.0:
            return
        self._last_value = value
        try:
            subprocess.Popen(
                ['gz', 'topic', '-t', self.topic, '-m', GZ_MSG_TYPE,
                 '-p', f'data:{value}'],
                stdout=self._devnull,
                stderr=self._devnull,
                preexec_fn=os.setpgrp,
            )
        except Exception:
            pass

    def close(self):
        try:
            self._devnull.close()
        except Exception:
            pass


class MavrosToVrxBridge(Node):
    """Converte comandos RC do MAVROS para força (Thrust) no Gazebo."""
    def __init__(self):
        super().__init__('mavros_to_vrx_bridge')

        # Parâmetros configuráveis
        self.declare_parameter('thrust_max', THRUST_MAX)
        self.declare_parameter('mavros_namespace', 'whiteboat')
        self.declare_parameter('steering_channel', 0)   # SERVO1 → Steering
        self.declare_parameter('throttle_channel', 2)   # SERVO3 → Throttle

        mavros_ns = normalize_namespace(self.get_parameter('mavros_namespace').value)
        self.steer_ch = int(self.get_parameter('steering_channel').value)
        self.throttle_ch = int(self.get_parameter('throttle_channel').value)
        self.thrust_max = float(self.get_parameter('thrust_max').value)
        self._override_active_until_ns = 0
        self._override_timeout_ns = int(0.5e9)

        if self.steer_ch < 0 or self.throttle_ch < 0:
            raise ValueError('steering_channel e throttle_channel devem ser >= 0.')

        # Publishers diretos via Gazebo Transport (bypass ros_gz_bridge)
        self.left_gz = GzPublisher(GZ_LEFT_THRUST)
        self.right_gz = GzPublisher(GZ_RIGHT_THRUST)

        # Subscriber nos outputs RC do MAVROS
        self.create_subscription(
            RCOut,
            f'/{mavros_ns}/rc/out',
            self.rc_out_callback,
            10
        )
        self.create_subscription(
            OverrideRCIn,
            f'/{mavros_ns}/rc/override',
            self.rc_override_callback,
            10
        )

        self.get_logger().info(
            f'Ponte MAVROS→VRX (GZ Transport direto) iniciada | '
            f'steering_ch={self.steer_ch} throttle_ch={self.throttle_ch} | '
            f'thrust_max={self.thrust_max} N | '
            f'RCOut=/{mavros_ns}/rc/out | Override=/{mavros_ns}/rc/override'
        )

    def _publish_thrust(self, throttle_pwm, steering_pwm):
        # PWM 0 = canal inativo / override liberado.
        if throttle_pwm == 0 and steering_pwm == 0:
            throttle_pwm = PWM_NEUTRAL
            steering_pwm = PWM_NEUTRAL

        if throttle_pwm in (0, 65535):
            throttle_pwm = PWM_NEUTRAL
        if steering_pwm in (0, 65535):
            steering_pwm = PWM_NEUTRAL

        throttle = pwm_to_normalized(throttle_pwm)  # [-1, 1]
        steering = pwm_to_normalized(steering_pwm)  # [-1, 1] (neg=esq, pos=dir)

        # Mixagem diferencial: rover-style
        #   left  = throttle + steering
        #   right = throttle - steering
        left_norm = max(-1.0, min(1.0, throttle + steering))
        right_norm = max(-1.0, min(1.0, throttle - steering))

        left_n = left_norm * self.thrust_max
        right_n = right_norm * self.thrust_max

        self.left_gz.publish(left_n)
        self.right_gz.publish(right_n)

        self.get_logger().info(
            f'PWM steer={steering_pwm} throttle={throttle_pwm} → '
            f'L={left_n:.1f}N R={right_n:.1f}N (GZ direto)',
            throttle_duration_sec=1
        )

    def _read_channels(self, channels):
        if len(channels) <= max(self.steer_ch, self.throttle_ch):
            return None

        throttle_pwm = channels[self.throttle_ch]
        steering_pwm = channels[self.steer_ch]
        return throttle_pwm, steering_pwm

    def rc_out_callback(self, msg: RCOut):
        if self.get_clock().now().nanoseconds < self._override_active_until_ns:
            return

        pwm = self._read_channels(msg.channels)
        if pwm is None:
            self.get_logger().warn(
                'RCOut sem canais suficientes para steering/throttle.',
                throttle_duration_sec=5.0,
            )
            return
        self._publish_thrust(*pwm)

    def rc_override_callback(self, msg: OverrideRCIn):
        pwm = self._read_channels(msg.channels)
        if pwm is None:
            self.get_logger().warn(
                'OverrideRCIn sem canais suficientes para steering/throttle.',
                throttle_duration_sec=5.0,
            )
            return

        throttle_pwm, steering_pwm = pwm
        override_active = throttle_pwm not in (0, 65535) or steering_pwm not in (0, 65535)
        if override_active:
            self._override_active_until_ns = self.get_clock().now().nanoseconds + self._override_timeout_ns
        self._publish_thrust(throttle_pwm, steering_pwm)


def main(args=None):
    rclpy.init(args=args)
    node = MavrosToVrxBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.left_gz.close()
        node.right_gz.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
