#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geographic_msgs.msg import GeoPoseStamped
# TODO: import whiteboat_sensors

class LocalizationNode:
    def __init__(self):
        self.GPS_TOPIC = '/whiteboat/mavros/sensors/gps/gps/fix' # TODO: integrate to wb_sensors
        self.IMU_TOPIC = '/whiteboat/mavros/sensors/imu/imu/data' # TODO: integrate to wb_sensors
        self.GEOPOSE_TOPIC = '/whiteboat/mavros/global_position/global' # TODO: integrate to wb_sensors

        self.latest_gps = None
        self.latest_imu = None

        self.geopose_pub = rospy.Publisher(self.GEOPOSE_TOPIC, GeoPoseStamped, queue_size=10)
        self.gps_sub = rospy.Subscriber(self.GPS_TOPIC, NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber(self.IMU_TOPIC, Imu, self.imu_callback)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Nó de localização iniciado.")
        rospy.loginfo("Aguardando GPS e IMU...")

    def gps_callback(self, msg):
        if msg.status.status >= 0:
            if self.latest_gps is None:
                rospy.loginfo("Mensagem de GPS recebido.")
            self.latest_gps = msg

    def imu_callback(self, msg):
        if self.latest_imu is None:
            rospy.loginfo("Mensagem de IMU recebido.")
        self.latest_imu = msg

    def run(self):
        if self.latest_gps is not None and self.latest_imu is not None:
            geo_pose_msg = GeoPoseStamped()

            geo_pose_msg.header.stamp = rospy.Time.now()
            geo_pose_msg.header.frame_id = 'map' # 'world'/'odom'
            geo_pose_msg.pose.position.latitude = self.latest_gps.latitude
            geo_pose_msg.pose.position.longitude = self.latest_gps.longitude
            geo_pose_msg.pose.position.altitude = self.latest_gps.altitude
            geo_pose_msg.pose.orientation = self.latest_imu.orientation
            
            self.geopose_pub.publish(geo_pose_msg)

    def shutdown(self):
        rospy.loginfo("Desligando LocalizationNode...")

def main():
    rospy.init_node('localization_node', anonymous=True)
    localization = LocalizationNode()
    rate = rospy.Rate(10) 

    try:
        while not rospy.is_shutdown():
            localization.run()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Programa interrompido (Ctrl+C).")
        pass

if __name__ == '__main__':
    main()