#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import math
from sensor_msgs.msg import Imu
from mavros_msgs.msg import OverrideRCIn, State
from tf_transformations import euler_from_quaternion


class SquareNode(Node):
    def __init__(self):
        super().__init__('square_node')

        self.RC_OVERRIDE_TOPIC = '/whiteboat/rc/override'
        self.IMU_TOPIC = '/whiteboat/imu/data'
        self.MODE_TOPIC = '/whiteboat/state'

        self.THRUST_NEUTRAL = 0.0
        self.THRUST_FORWARD = 20.0
        self.SIDE_DURATION = 4.0
        self.WAIT_DURATION = 2.0

        self.THRUST_TURN = 1600
        self.DEGREE_RANGE = 5.0

        self.RC_MIN = 1450
        self.RC_NEUTRAL = 1500
        self.RC_MAX = 1550
        self.THROTTLE_CHANNEL = 2
        self.STEERING_CHANNEL = 0

        from rclpy.qos import qos_profile_sensor_data
        self.rc_override_pub = self.create_publisher(OverrideRCIn, self.RC_OVERRIDE_TOPIC, 1)
        self.imu_sub = self.create_subscription(Imu, self.IMU_TOPIC, self.imu_callback, qos_profile_sensor_data)
        self.mode_sub = self.create_subscription(State, self.MODE_TOPIC, self.mode_callback, 10)

        self.get_logger().info('Aguardando IMU...')
        self.current_yaw = None
        self.target_yaw = None

        self.last_imu_time = None
        self.imu_timeout = Duration(seconds=1.0)

        self.state = 'WAITING_FOR_IMU'
        self.state_start_time = None
        self.side_counter = 0

        # Flags para simular loginfo_once
        self._logged_waiting_imu = False
        self._logged_done = False
        self._logged_imu_failure = False
        self._logged_incorrect_mode = False

        # Timer a 20 Hz
        self.timer = self.create_timer(1.0 / 20.0, self.run)

    def mode_callback(self, data: State):
        if data.mode == 'MANUAL':
            if self.state == 'WAITING_FOR_MODE':
                self.change_state('FORWARD')
        else:
            self.change_state('INCORRECT_MODE')

    def imu_callback(self, msg: Imu):
        self.last_imu_time = self.get_clock().now()
        orientation_q = msg.orientation
        _, _, self.current_yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        if self.state == 'WAITING_FOR_IMU':
            self.get_logger().info('Dados da IMU recebidos!')
            self.change_state('WAITING_FOR_MODE')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def set_thrusters(self, left_thrust, right_thrust):
        throttle_scaled = (left_thrust + right_thrust) / 2.0
        steering_scaled = (right_thrust - left_thrust) / 2.0

        throttle_pwm = self.RC_NEUTRAL + (throttle_scaled / self.THRUST_FORWARD) * (self.RC_MAX - self.RC_NEUTRAL)
        steering_pwm = self.RC_NEUTRAL + (steering_scaled / self.THRUST_FORWARD) * (self.RC_MAX - self.RC_NEUTRAL)

        throttle_pwm = max(self.RC_MIN, min(self.RC_MAX, throttle_pwm))
        steering_pwm = max(self.RC_MIN, min(self.RC_MAX, steering_pwm))

        rc_msg = OverrideRCIn()
        rc_msg.channels = [65535] * 18
        rc_msg.channels[self.THROTTLE_CHANNEL] = int(throttle_pwm)
        rc_msg.channels[self.STEERING_CHANNEL] = int(steering_pwm)

        self.rc_override_pub.publish(rc_msg)

    def release_control(self):
        self.get_logger().info('Liberando controle RC...')
        rc_msg = OverrideRCIn()
        rc_msg.channels = [0] * 18
        self.rc_override_pub.publish(rc_msg)

    def stopping_the_boat(self):
        self.set_thrusters(0.0, 0.0)
        # TODO: call loiter mode service

    def shutdown(self):
        self.get_logger().info('Barco parado, desligando o nó.')
        self.stopping_the_boat()
        self.release_control()

    def change_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(f'Mudando de estado: {self.state} -> {new_state}')
            self.state = new_state

            if self.state == 'FORWARD':
                self.state_start_time = self.get_clock().now()
                self.get_logger().info(
                    f'#--- LADO {self.side_counter + 1}: Iniciando movimento em linha reta... ---#'
                )

            elif self.state == 'TURN':
                self.target_yaw = self.normalize_angle(self.current_yaw - math.radians(90))

            elif self.state == 'WAIT_TO_STOP':
                self.state_start_time = self.get_clock().now()
                self.get_logger().info(
                    f'#--- Aguardando {self.WAIT_DURATION} segundos para o barco parar... ---#'
                )

    def run(self):
        if self.state == 'WAITING_FOR_IMU':
            if not self._logged_waiting_imu:
                self.get_logger().info('Aguardando a primeira mensagem da IMU...')
                self._logged_waiting_imu = True
            return

        if self.last_imu_time is None or \
                (self.get_clock().now() - self.last_imu_time) > self.imu_timeout:
            self.change_state('IMU_FAILURE')
            raise Exception('ERRO: Dados da IMU não estão sendo recebidos!')

        if self.state == 'FORWARD':
            self.set_thrusters(self.THRUST_FORWARD, self.THRUST_FORWARD)
            if self.state_start_time is None:
                raise Exception('ERRO: Faltando o start time!')
            elif (self.get_clock().now() - self.state_start_time) >= Duration(seconds=self.SIDE_DURATION):
                self.side_counter += 1
                if self.side_counter >= 4:
                    self.change_state('DONE')
                else:
                    self.change_state('WAIT_TO_STOP')

        elif self.state == 'TURN':
            error_rad = self.normalize_angle(self.target_yaw - self.current_yaw)
            error_degrees = math.degrees(error_rad)
            self.get_logger().info(
                f'Alvo: {math.degrees(self.target_yaw):.1f}, '
                f'Atual: {math.degrees(self.current_yaw):.1f}, '
                f'Erro: {error_degrees:.1f}°',
                throttle_duration_sec=1
            )
            if abs(error_degrees) <= self.DEGREE_RANGE:
                self.change_state('FORWARD')
            else:
                self.set_thrusters(self.THRUST_TURN, -self.THRUST_TURN)

        elif self.state == 'WAIT_TO_STOP':
            self.stopping_the_boat()
            if self.state_start_time is None:
                raise Exception('ERRO: Faltando o start time!')
            elif (self.get_clock().now() - self.state_start_time) >= Duration(seconds=self.WAIT_DURATION):
                self.change_state('TURN')

        elif self.state == 'DONE':
            if not self._logged_done:
                self.get_logger().info('Percurso do quadrado finalizado!')
                self._logged_done = True
            self.stopping_the_boat()
            self.release_control()

        elif self.state == 'IMU_FAILURE':
            if not self._logged_imu_failure:
                self.get_logger().error('Falha da IMU!')
                self._logged_imu_failure = True
            self.shutdown()

        elif self.state == 'INCORRECT_MODE':
            if not self._logged_incorrect_mode:
                self.get_logger().warn('Modo de voo incorreto!')
                self._logged_incorrect_mode = True
            self.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SquareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Parando o barco...')
        node.stopping_the_boat()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
