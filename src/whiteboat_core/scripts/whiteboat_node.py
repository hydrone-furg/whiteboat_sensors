#!/usr/bin/env python3
import rospy
import math
import time
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from std_msgs.msg import Header 
from whiteboat_tools.common import MAVLinkConnection
from pymavlink.dialects.v10.ardupilotmega import MAVLink_gps_raw_int_message, MAVLink_scaled_imu2_message, MAVLink_raw_imu_message, MAVLink_sys_status_message, MAVLink_attitude_message, MAVLink_highres_imu_message
from pymavlink import mavutil
from tf.transformations import quaternion_from_euler

class WhiteboatNode:
    def __init__(self):
        rospy.init_node('whiteboat_node', anonymous=False)
        rospy.loginfo("Whiteboat main node started.")

        self.gps_publisher = rospy.Publisher('/whiteboat/gps/fix', NavSatFix, queue_size=10)
        self.imu_publisher = rospy.Publisher('/whiteboat/imu/data', Imu, queue_size=10)
        self.rate = rospy.Rate(1)  # 1 Hz
        self.latest_attitude = None
        self.threshold = 1.4

        self.gps_msg = NavSatFix()
        self.gps_msg.status.status = 0  # 0 = FIX, -1 = NO_FIX
        self.gps_msg.status.service = 1  # GPS service type

        self.mav = MAVLinkConnection(sitl_address='udp:127.0.0.1:14550', simulating=True) # para usar via SITL
        #self.mav = MAVLinkConnection(baud=57600, simulating=False)
        self.mav.connect()
        self.motor_test()

        self.mav.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 10)
        self.mav.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)
        
        
        while not rospy.is_shutdown():
            msg = self.mav.read_sensors()
            if msg:
                msg_time = time.time() - msg._timestamp
                if msg_time > self.threshold:
                    rospy.logwarn("Lost message -> MAVLink message: %s (%.2f seconds)", msg.get_type(), msg_time)
                    continue
                if msg.get_type() in ['ATTITUDE']:
                    self.latest_attitude = msg

                elif msg.get_type() in ['RAW_IMU']:
                    if self.latest_attitude:
                        self.publish_imu(msg, self.latest_attitude)

                elif msg.get_type() in ['GPS_RAW_INT']:
                    # rospy.loginfo("Received GPS message")
                    self.publish_gps(msg)

    def motor_test(self):
        """
        Arm, disarm, and test the actuation of the thrusters.
        """
        try:
            time.sleep(2)
            mode_id = self.mav.connection.mode_mapping()['ACRO']
            self.mav.connection.set_mode(mode_id)
            time.sleep(2)

            self.mav.arm() # arming

            MOTOR_1 = 1 # SERVO1_FUNCTION (73) > Throttle Left
            MOTOR_3 = 3 # SERVO3_FUNCTION (74) > Throttle Right

            rospy.loginfo("MOTOR_3: 1550")
            self.mav.set_rc_channel_pwm(MOTOR_3, 1550)
            time.sleep(3)
            rospy.loginfo("MOTOR_3: 1500")
            self.mav.set_rc_channel_pwm(MOTOR_3, 1500)
            rospy.loginfo("MOTOR_1: 1550")
            self.mav.set_rc_channel_pwm(MOTOR_1, 1550)
            time.sleep(3)
            rospy.loginfo("MOTOR_1: 1500")
            self.mav.set_rc_channel_pwm(MOTOR_1, 1500)
            time.sleep(1)

        except Exception as e:
            rospy.logerr(f"Error: {e}")
        finally:
            self.mav.clear_rc_override() # back to manual control
            rospy.loginfo("Desarming...")
            time.sleep(2)
            self.mav.disarm()
            rospy.loginfo("Disarmed.")
    

    def publish_gps(self, msg: MAVLink_gps_raw_int_message):
        """
        Parses, and publishes GPS message.

        Args:
            msg (MAVLink_message): GPS message.
        """
        gps_msg = NavSatFix()
        gps_msg.altitude = msg.alt / 1e3
        gps_msg.latitude = msg.lat / 1e7
        gps_msg.longitude = msg.lon / 1e7
        

        # TODO: check covariance math
        h_acc = msg.h_acc / 1000  # Horizontal accuracy (meters)
        v_acc = msg.v_acc / 1000  # Vertical accuracy (meters)
        gps_msg.position_covariance = [
            h_acc**2, 0, 0,
            0, h_acc**2, 0,
            0, 0, v_acc**2
        ]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.gps_msg.header = Header(frame_id='gps_frame')
        gps_msg.header.stamp = rospy.Time.now()
        if msg.fix_type >= 3:
            gps_msg.status.status = NavSatStatus.STATUS_FIX
        else:
            gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        self.gps_publisher.publish(gps_msg)


    def publish_imu(self, imu_msg_raw: MAVLink_raw_imu_message, attitude_msg: MAVLink_attitude_message):
        """
        Combine and publish the IMU message.

        Args:
            imu_msg_raw (MAVLink_message): The raw IMU readings.
            attitude_msg (MAVLink_message): The attitude in the aeronautical frame.
        """
        imu_msg = Imu()
        imu_msg.header = Header(frame_id='imu_link')
        imu_msg.header.stamp = rospy.Time.now()

        # -- Orientation --
        q = quaternion_from_euler(
            attitude_msg.roll,
            attitude_msg.pitch,
            attitude_msg.yaw
        )
        imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # -- Angular velocity --
        imu_msg.angular_velocity = Vector3(
            x=imu_msg_raw.xgyro / 1000.0,
            y=imu_msg_raw.ygyro / 1000.0,
            z=imu_msg_raw.zgyro / 1000.0
        )
        # -- Linear acceleration --
        imu_msg.linear_acceleration = Vector3(
            x=(imu_msg_raw.xacc / 1000.0) * 9.8066,
            y=(imu_msg_raw.yacc / 1000.0) * 9.8066,
            z=(imu_msg_raw.zacc / 1000.0) * 9.8066
        )
        self.imu_publisher.publish(imu_msg)


if __name__ == '__main__':
    try:
        node = WhiteboatNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
