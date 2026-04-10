#!/usr/bin/env python3

import threading
import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler

from whiteboat_tools.common import MAVLinkConnection
from pymavlink.dialects.v10.ardupilotmega import (
    MAVLink_gps_raw_int_message,
    MAVLink_raw_imu_message,
    MAVLink_attitude_message,
)
from pymavlink import mavutil


class WhiteboatNode(Node):
    def __init__(self):
        super().__init__('whiteboat_node')
        self.get_logger().info('Whiteboat main node started.')

        self.gps_publisher = self.create_publisher(NavSatFix, '/whiteboat/gps/fix', 10)
        self.imu_publisher = self.create_publisher(Imu, '/whiteboat/imu/data', 10)

        self.latest_attitude = None
        self.threshold = 1.4

        self.gps_msg = NavSatFix()
        self.gps_msg.status.status = 0   # 0 = FIX
        self.gps_msg.status.service = 1  # GPS service type

        # Conexão MAVLink em thread separada para não bloquear o executor ROS 2
        self.mav = MAVLinkConnection(sitl_address='udp:127.0.0.1:14550', simulating=True)
        # self.mav = MAVLinkConnection(baud=57600, simulating=False)
        self.mav.connect()
        self.motor_test()

        self.mav.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 10)
        self.mav.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)

        # Thread que faz a leitura contínua de sensores MAVLink
        self._mav_thread = threading.Thread(target=self._mav_loop, daemon=True)
        self._mav_thread.start()

    def _mav_loop(self):
        """Loop de leitura de mensagens MAVLink em thread separada."""
        while rclpy.ok():
            msg = self.mav.read_sensors()
            if not msg:
                continue

            msg_time = time.time() - msg._timestamp
            if msg_time > self.threshold:
                self.get_logger().warn(
                    f'Lost message → MAVLink message: {msg.get_type()} ({msg_time:.2f}s)'
                )
                continue

            if msg.get_type() == 'ATTITUDE':
                self.latest_attitude = msg

            elif msg.get_type() == 'RAW_IMU':
                if self.latest_attitude:
                    self.publish_imu(msg, self.latest_attitude)

            elif msg.get_type() == 'GPS_RAW_INT':
                self.publish_gps(msg)

    def motor_test(self):
        """Arma, testa atuação dos propulsores e desarma."""
        try:
            time.sleep(2)
            mode_id = self.mav.connection.mode_mapping()['ACRO']
            self.mav.connection.set_mode(mode_id)
            time.sleep(2)

            self.mav.arm()

            MOTOR_1 = 1  # SERVO1_FUNCTION (73) > Throttle Left
            MOTOR_3 = 3  # SERVO3_FUNCTION (74) > Throttle Right

            self.get_logger().info('MOTOR_3: 1550')
            self.mav.set_rc_channel_pwm(MOTOR_3, 1550)
            time.sleep(3)
            self.get_logger().info('MOTOR_3: 1500')
            self.mav.set_rc_channel_pwm(MOTOR_3, 1500)
            self.get_logger().info('MOTOR_1: 1550')
            self.mav.set_rc_channel_pwm(MOTOR_1, 1550)
            time.sleep(3)
            self.get_logger().info('MOTOR_1: 1500')
            self.mav.set_rc_channel_pwm(MOTOR_1, 1500)
            time.sleep(1)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            self.mav.clear_rc_override()
            self.get_logger().info('Desarming...')
            time.sleep(2)
            self.mav.disarm()
            self.get_logger().info('Disarmed.')

    def publish_gps(self, msg: MAVLink_gps_raw_int_message):
        """Parseia e publica mensagem GPS."""
        gps_msg = NavSatFix()
        gps_msg.altitude = msg.alt / 1e3
        gps_msg.latitude = msg.lat / 1e7
        gps_msg.longitude = msg.lon / 1e7

        h_acc = msg.h_acc / 1000
        v_acc = msg.v_acc / 1000
        gps_msg.position_covariance = [
            h_acc**2, 0, 0,
            0, h_acc**2, 0,
            0, 0, v_acc**2
        ]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        gps_msg.header.frame_id = 'gps_frame'
        gps_msg.header.stamp = self.get_clock().now().to_msg()

        if msg.fix_type >= 3:
            gps_msg.status.status = NavSatStatus.STATUS_FIX
        else:
            gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS

        self.gps_publisher.publish(gps_msg)

    def publish_imu(self, imu_msg_raw: MAVLink_raw_imu_message, attitude_msg: MAVLink_attitude_message):
        """Combina e publica mensagem IMU."""
        imu_msg = Imu()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.header.stamp = self.get_clock().now().to_msg()

        # Orientação
        q = quaternion_from_euler(
            attitude_msg.roll,
            attitude_msg.pitch,
            attitude_msg.yaw
        )
        imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Velocidade angular
        imu_msg.angular_velocity = Vector3(
            x=imu_msg_raw.xgyro / 1000.0,
            y=imu_msg_raw.ygyro / 1000.0,
            z=imu_msg_raw.zgyro / 1000.0
        )

        # Aceleração linear
        imu_msg.linear_acceleration = Vector3(
            x=(imu_msg_raw.xacc / 1000.0) * 9.8066,
            y=(imu_msg_raw.yacc / 1000.0) * 9.8066,
            z=(imu_msg_raw.zacc / 1000.0) * 9.8066
        )

        self.imu_publisher.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WhiteboatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
