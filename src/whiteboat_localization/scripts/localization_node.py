#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geographic_msgs.msg import GeoPoseStamped
# TODO: import whiteboat_sensors


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')

        self.GPS_TOPIC = '/whiteboat/mavros/sensors/gps/gps/fix'   # TODO: integrate to wb_sensors
        self.IMU_TOPIC = '/whiteboat/mavros/sensors/imu/imu/data'   # TODO: integrate to wb_sensors
        self.GEOPOSE_TOPIC = '/whiteboat/mavros/global_position/global'  # TODO: integrate to wb_sensors

        self.latest_gps = None
        self.latest_imu = None

        self.geopose_pub = self.create_publisher(GeoPoseStamped, self.GEOPOSE_TOPIC, 10)
        self.gps_sub = self.create_subscription(NavSatFix, self.GPS_TOPIC, self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, self.IMU_TOPIC, self.imu_callback, 10)

        # Timer a 10 Hz
        self.timer = self.create_timer(1.0 / 10.0, self.run)

        self.get_logger().info('Nó de localização iniciado.')
        self.get_logger().info('Aguardando GPS e IMU...')

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status >= 0:
            if self.latest_gps is None:
                self.get_logger().info('Mensagem de GPS recebido.')
            self.latest_gps = msg

    def imu_callback(self, msg: Imu):
        if self.latest_imu is None:
            self.get_logger().info('Mensagem de IMU recebido.')
        self.latest_imu = msg

    def run(self):
        if self.latest_gps is not None and self.latest_imu is not None:
            geo_pose_msg = GeoPoseStamped()

            geo_pose_msg.header.stamp = self.get_clock().now().to_msg()
            geo_pose_msg.header.frame_id = 'map'  # 'world'/'odom'
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