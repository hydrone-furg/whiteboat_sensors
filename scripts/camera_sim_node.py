#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from mavros_msgs.msg import State

class CameraSimNode:
    def __init__(self):
        # Tópico padrão do VRX 
        self.SIM_CAMERA_TOPIC = '/wamv/sensors/cameras/front_left_camera/image_raw'
        
        # Tópicos do Whiteboat
        self.MY_CAMERA_TOPIC = '/sensors/camera/image_raw'
        self.MAVROS_STATE_TOPIC = '/mavros/state'

        # Publishers e Subscribers
        self.image_pub = rospy.Publisher(self.MY_CAMERA_TOPIC, Image, queue_size=10)
        
        # Recebe a imagem e o estado
        self.sim_image_sub = rospy.Subscriber(self.SIM_CAMERA_TOPIC, Image, self.sim_img_callback)
        self.mavros_state_sub = rospy.Subscriber(self.MAVROS_STATE_TOPIC, State, self.state_callback)

        self.mavros_connected = False
        self.current_mode = "UNKNOWN"

        rospy.loginfo("Nó de Simulação de Câmera iniciado.")
        rospy.loginfo(f"Escutando: {self.SIM_CAMERA_TOPIC}")
        rospy.loginfo(f"Publicando em: {self.MY_CAMERA_TOPIC}")

    def state_callback(self, msg: State):
        self.mavros_connected = msg.connected
        self.current_mode = msg.mode

    def sim_img_callback(self, msg: Image):
       
        if self.mavros_connected:
            
            msg.header.stamp = rospy.Time.now()
          
            # Publica no tópico /sensors/...
            self.image_pub.publish(msg)
        else:
            pass 

    def run(self):
        rospy.spin()

def main():
    rospy.init_node('camera_sim_node', anonymous=True)
    node = CameraSimNode()
    node.run()

if __name__ == '__main__':
    main()