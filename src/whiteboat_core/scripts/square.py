#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Imu
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import State
from tf.transformations import euler_from_quaternion
#from rostopic import ROSTopicHz


class SquareNode:
    def __init__(self):
        self.RC_OVERRIDE_TOPIC = '/mavros/rc/override'
        self.IMU_TOPIC = '/mavros/imu/data'
        self.MODE_TOPIC = '/whiteboat/mavros/state'

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

        self.rc_override_pub = rospy.Publisher(self.RC_OVERRIDE_TOPIC, OverrideRCIn, queue_size=1)
        self.imu_sub = rospy.Subscriber(self.IMU_TOPIC, Imu, self.imu_callback)
        self.mode_topic = rospy.Subscriber(self.MODE_TOPIC, State, self.mode_callback)
        '''
        self.MIN_IMU_HZ = 10.0
        self.imu_rate_monitor = ROSTopicHz(15, filter_expr=None)
        self.imu_rate_sub = rospy.Subscriber(self.IMU_TOPIC, rospy.AnyMsg, self.imu_rate_monitor.callback_hz)
        '''
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Aguardando IMU...")
        self.current_yaw  = None
        self.target_yaw = None
        # self.imu_received = False

        self.last_imu_time = None
        self.imu_timeout = rospy.Duration(1.0)

        self.state = 'WAITING_FOR_IMU'
        self.state_start_time = None
        self.side_counter = 0

    def mode_callback(self, data):
        if data.mode == "MANUAL": 
            if self.state == "WAITING_FOR_MODE":
                self.change_state("FORWARD")
        else:
            self.change_state("INCORRECT_MODE")

    def imu_callback(self, msg: Imu):
        self.last_imu_time = rospy.Time.now()
        orientation_q = msg.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        if self.state == 'WAITING_FOR_IMU':
            rospy.loginfo("Dados da IMU recebidos!")
            self.change_state('WAITING_FOR_MODE')

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
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
        rospy.loginfo_once("Liberando controle RC...")
        rc_msg = OverrideRCIn()
        rc_msg.channels = [0] * 18
        self.rc_override_pub.publish(rc_msg)

    def stopping_the_boat(self):
        self.set_thrusters(0.0, 0.0)
        # TODO: call loiter mode service

    def shutdown(self):
        rospy.loginfo_once("Barco parado, desligando o nó.")
        self.stopping_the_boat()
        self.release_control()

    def change_state(self, new_state):
        if self.state != new_state:
            rospy.loginfo(f"Mudando de estado: {self.state} -> {new_state}")
            self.state = new_state

            if self.state == 'FORWARD':
                self.state_start_time = rospy.Time.now()
                rospy.loginfo(f"#--- LADO {self.side_counter + 1}: Iniciando movimento em linha reta... ---#")
            
            elif self.state == 'TURN':
                self.target_yaw = self.normalize_angle(self.current_yaw - math.radians(90))

            elif self.state == 'WAIT_TO_STOP':
                self.state_start_time = rospy.Time.now()
                rospy.loginfo_once(f"#--- Aguardando {self.WAIT_DURATION} segundos para o barco parar... ---#")

    def run(self):
        #hz_info = self.imu_rate_monitor.get_hz(self.IMU_TOPIC)

        
        #if self.imu_received and current_rate < self.MIN_IMU_HZ and self.state != 'IMU_FAILURE':
            #rospy.loginfo(f"FREQUÊNCIA DA IMU BAIXA: {current_rate:.2f} Hz. Parando o barco!")
            #self.change_state('IMU_FAILURE')
        
        if self.state == 'WAITING_FOR_IMU':
            rospy.loginfo_once("Aguardando a primeira mensagem da IMU...")
            return

        if self.last_imu_time is None or (rospy.Time.now() - self.last_imu_time) > self.imu_timeout:
            self.change_state('IMU_FAILURE')
            raise Exception("ERRO: Dados da IMU não estão sendo recebidos!")
        

        if self.state == 'FORWARD':
            self.set_thrusters(self.THRUST_FORWARD, self.THRUST_FORWARD)
            # TODO: call manual mode service
            if self.state_start_time is None:
                raise Exception("ERRO: Faltando o start time!")

            elif (rospy.Time.now() - self.state_start_time >= rospy.Duration(self.SIDE_DURATION)):
                self.side_counter += 1
                if self.side_counter >= 4:
                    self.change_state('DONE')
                else:
                    self.change_state('WAIT_TO_STOP')

        elif self.state == 'TURN':
            # TODO: call manual mode service
            error_rad = self.normalize_angle(self.target_yaw - self.current_yaw)
            error_degrees = math.degrees(error_rad)
            rospy.loginfo_throttle(1, f"Alvo: {math.degrees(self.target_yaw):.1f}, Atual: {math.degrees(self.current_yaw):.1f}, Erro: {error_degrees:.1f}°")
            if abs(error_degrees) <= self.DEGREE_RANGE:
                self.change_state('FORWARD')
            else:
                self.set_thrusters(self.THRUST_TURN, -self.THRUST_TURN)
                
        elif self.state == 'WAIT_TO_STOP':
            self.stopping_the_boat()
            
            if self.state_start_time is None:
                raise Exception("ERRO: Faltando o start time!")

            elif (rospy.Time.now() - self.state_start_time >= rospy.Duration(self.WAIT_DURATION)):
                self.change_state('TURN')
        
        elif self.state == 'DONE':
            rospy.loginfo_once("Percurso do quadrado finalizado!")
            self.stopping_the_boat()
            self.release_control()

        elif self.state == 'IMU_FAILURE':
            rospy.loginfo_once("Falha da IMU!")
            self.shutdown()
        
        # imu failed in operation condition
        elif self.state == 'INCORRECT_MODE':
            rospy.loginfo_once("Modo de voo incorreto!")
            self.shutdown()


def main():
    rospy.init_node('square_node', anonymous=True)
    rospy.loginfo("Nó 'square_node' inicializado.")
    controller = SquareNode()
    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            controller.run()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Programa interrompido (Ctrl+C).")
        pass

    finally:
        rospy.loginfo('Parando o barco...')
        controller.stopping_the_boat()
        
if __name__ == '__main__':
    main()
