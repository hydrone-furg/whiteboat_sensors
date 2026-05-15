#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geographic_msgs.msg import GeoPoseStamped


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')

        self.declare_parameter('gps_topic', '/whiteboat/global_position/global')
        self.declare_parameter('imu_topic', '/whiteboat/imu/data')
        self.declare_parameter('geopose_topic', '/whiteboat/geopose')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 5.0)

        self.GPS_TOPIC = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.IMU_TOPIC = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.GEOPOSE_TOPIC = self.get_parameter('geopose_topic').get_parameter_value().string_value
        self.FRAME_ID = self.get_parameter('frame_id').get_parameter_value().string_value
        self.PUBLISH_RATE_HZ = max(0.0, float(self.get_parameter('publish_rate_hz').value))
        self._min_publish_period_ns = int(1e9 / self.PUBLISH_RATE_HZ) if self.PUBLISH_RATE_HZ > 0.0 else 0
        self._last_publish_ns = 0

        self.latest_gps = None
        self.latest_imu = None

        from rclpy.qos import qos_profile_sensor_data

        self.geopose_pub = self.create_publisher(GeoPoseStamped, self.GEOPOSE_TOPIC, 1)
        self.gps_sub = self.create_subscription(NavSatFix, self.GPS_TOPIC, self.gps_callback, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(Imu, self.IMU_TOPIC, self.imu_callback, qos_profile_sensor_data)

        self.get_logger().info('Nó de localização iniciado.')
        self.get_logger().info('Aguardando GPS e IMU...')

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            if self.latest_gps is None:
                self.get_logger().info('Mensagem de GPS recebido.')
            self.latest_gps = msg
            self.publish_geopose_if_ready()

    def imu_callback(self, msg: Imu):
        if self.latest_imu is None:
            self.get_logger().info('Mensagem de IMU recebido.')
        self.latest_imu = msg

    def publish_geopose_if_ready(self):
        if self.latest_gps is not None and self.latest_imu is not None:
            now = self.get_clock().now()
            now_ns = now.nanoseconds
            if (
                self._min_publish_period_ns > 0 and
                now_ns - self._last_publish_ns < self._min_publish_period_ns
            ):
                return
            self._last_publish_ns = now_ns

            geo_pose_msg = GeoPoseStamped()

            geo_pose_msg.header.stamp = now.to_msg()
            geo_pose_msg.header.frame_id = self.FRAME_ID
            geo_pose_msg.pose.position.latitude = self.latest_gps.latitude
            geo_pose_msg.pose.position.longitude = self.latest_gps.longitude
            geo_pose_msg.pose.position.altitude = self.latest_gps.altitude
            geo_pose_msg.pose.orientation = self.latest_imu.orientation

            self.geopose_pub.publish(geo_pose_msg)

    def shutdown(self):
        self.get_logger().info('Desligando LocalizationNode...')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
