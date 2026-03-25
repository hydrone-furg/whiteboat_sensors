#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from mavros_msgs.msg import State

class CameraSimNode:
    def __init__(self):
        # Tópico padrão do VRX
        self.SIM_CAMERA_TOPIC = '/wamv/sensors/cameras/front_left_camera/image_raw'
        
        # Tópicos do Whiteboat
        self.CAMERA_TOPIC = '/whiteboat/sensors/camera/image_raw'
        self.MAVROS_STATE_TOPIC = '/whiteboat/mavros/state'

        # Inicializa o CvBridge
        self.bridge = CvBridge()

        # Publisher
        self.image_pub = rospy.Publisher(self.CAMERA_TOPIC, Image, queue_size=10)
        
        # Subscribers
        self.sim_image_sub = rospy.Subscriber(self.SIM_CAMERA_TOPIC, Image, self.sim_img_callback)
        self.mavros_state_sub = rospy.Subscriber(self.MAVROS_STATE_TOPIC, State, callback = self.state_callback)


        rospy.loginfo("Nó de Processamento de Câmera iniciado.")

    def state_callback(self, msg: State):
        # Debug do estado MAVROS
        self.mavros_connected = msg.connected
        rospy.loginfo_once(f'MAVROS Connected: {self.mavros_connected}')
        self.current_mode = msg.mode
        rospy.loginfo_once(f'MAVROS Mode: {self.current_mode}')

    def sim_img_callback(self, msg: Image):
        try:
            # Converte ROS Image message para OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Erro ao converter imagem: {e}")
            return

        # Processamento com OpenCV aqui
        (h, w) = cv_image.shape[:2]
        center = (w // 2, h // 2)
        
        # Desenha um círculo no centro da imagem
        cv2.circle(cv_image, center, 50, (0, 255, 0), 3)
        
        # Adiciona um texto na tela
        cv2.putText(cv_image, f"Mode: {self.current_mode}", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Debug local (descomente para ver a imagem)
        #cv2.imshow("Debug VRX", cv_image)
        #cv2.waitKey(1)

        # Converte OpenCV Image para ROS Image message para publicar
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            processed_msg.header = msg.header 
            
            self.image_pub.publish(processed_msg)

            # Debug de publicação (cuidado, pode travar)
            rospy.loginfo_once(f"{processed_msg} - Imagem processada publicada.")
        
        except CvBridgeError as e:
            rospy.logerr(f"Erro ao converter de volta para ROS: {e}")

    def run(self):
        rospy.spin()

def main():
    rospy.init_node('camera_sim_node', anonymous=True)
    node = CameraSimNode()
    node.run()

if __name__ == '__main__':
    main()