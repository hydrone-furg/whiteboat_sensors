#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from mavros_msgs.msg import State

class WhiteboatCam:
    def __init__(self):
        self.MAVROS_STATE_TOPIC = 'whiteboat/mavros/state'
        self.CAMERA_TOPIC = 'whiteboat/sensors/camera/image_raw'
        
        # Aqui fica as configurações da câmera
        self.CAMERA_INDEX = 0 # Pelo que eu vi, geralmente fica na porta 0
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 480
        
        # Incializa os Publishers e Subscribers
        self.image_pub = rospy.Publisher(self.CAMERA_TOPIC, Image, queue_size=10)
        self.mavros_state_sub = rospy.Subscriber(self.MAVROS_STATE_TOPIC, State, self.state_callback)

        # Inicializa a câmera
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.CAMERA_INDEX)
        
        # Configura a resolução da captura
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_HEIGHT)

        # Controle de Estado (copiei do teu código 'square')
        self.mavros_connected = False
        self.current_mode = "UNKNOWN"
        
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Nó de Câmera iniciado. Aguardando conexão com MAVROS...")

    def state_callback(self, msg: State):
        self.mavros_connected = msg.connected
        self.current_mode = msg.mode

    # Verifica se está tudo certo e converte Opencv em Ros message
    def capture_and_publish(self):
        if not self.cap.isOpened():
            rospy.logerr_throttle(5, "ERRO: Não foi possível acessar a câmera.")
            return

        ret, frame = self.cap.read()
        
        if ret:
            try:
                # Converte a imagem do OpenCV para Ros
                ros_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image_msg.header.stamp = rospy.Time.now()
                ros_image_msg.header.frame_id = "camera_link"

                self.image_pub.publish(ros_image_msg)
                
            except CvBridgeError as e:
                rospy.logerr(f"Erro na conversão CvBridge: {e}")
        else:
            rospy.logwarn_throttle(2, "Falha ao capturar frame da câmera.")

    # Desliga o Node
    def shutdown(self):
        rospy.loginfo_once("Desligando câmera e liberando recursos...")
        if self.cap.isOpened():
            self.cap.release()

    def run(self):
        if not self.mavros_connected:
            rospy.loginfo_throttle(5, "Aguardando conexão com a Pixhawk via MAVROS...")
            return

        self.capture_and_publish()

def main():
    rospy.init_node('camera_driver_node', anonymous=True)
    
    # Frequência de publicação (30 FPS)
    rate = rospy.Rate(30) 
    
    camera = WhiteboatCam()

    try:
        while not rospy.is_shutdown():
            camera.run()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        camera.shutdown()

if __name__ == '__main__':
    main()