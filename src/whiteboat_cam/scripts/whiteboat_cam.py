#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from mavros_msgs.msg import State


class WhiteboatCam(Node):
    def __init__(self):
        super().__init__('camera_driver_node')

        self.MAVROS_STATE_TOPIC = '/whiteboat/mavros/state'
        self.CAMERA_TOPIC = '/whiteboat/sensors/camera/image_raw'

        # Parâmetros configuráveis via launch ou CLI
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('wait_for_mavros', True)

        self.CAMERA_INDEX = self.get_parameter('camera_index').value
        self.WAIT_FOR_MAVROS = self.get_parameter('wait_for_mavros').value
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 480

        # Publishers e Subscribers
        self.image_pub = self.create_publisher(Image, self.CAMERA_TOPIC, 10)
        self.mavros_state_sub = self.create_subscription(
            State, self.MAVROS_STATE_TOPIC, self.state_callback, 10
        )

        # Inicializa a câmera
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_HEIGHT)

        # Controle de estado
        self.mavros_connected = False
        self.current_mode = 'UNKNOWN'
        self._mode_logged = False

        # Timer a 30 FPS
        self.timer = self.create_timer(1.0 / 30.0, self.run)

        self.get_logger().info('Nó de Câmera iniciado. Aguardando conexão com MAVROS...')

    def state_callback(self, msg: State):
        self.mavros_connected = msg.connected
        if not self._mode_logged:
            self.get_logger().info(f'Modo atual: {msg.mode}')
            self._mode_logged = True

    def capture_and_publish(self):
        if not self.cap.isOpened():
            self.get_logger().error(
                'ERRO: Não foi possível acessar a câmera.',
                throttle_duration_sec=5
            )
            return

        ret, frame = self.cap.read()

        if ret:
            try:
                ros_image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                ros_image_msg.header.stamp = self.get_clock().now().to_msg()
                ros_image_msg.header.frame_id = 'camera_link'
                self.image_pub.publish(ros_image_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'Erro na conversão CvBridge: {e}')
        else:
            self.get_logger().warn(
                'Falha ao capturar frame da câmera.',
                throttle_duration_sec=2
            )

    def show_image(self, frame):
        cv2.imshow('Whiteboat Camera', frame)
        cv2.waitKey(1)

    def shutdown(self):
        self.get_logger().info('Desligando câmera e liberando recursos...')
        if self.cap.isOpened():
            self.cap.release()

    def run(self):
        if self.WAIT_FOR_MAVROS and not self.mavros_connected:
            self.get_logger().info(
                'Aguardando conexão com a Pixhawk via MAVROS...',
                throttle_duration_sec=5
            )
            return
        self.capture_and_publish()


def main(args=None):
    rclpy.init(args=args)
    camera = WhiteboatCam()
    try:
        rclpy.spin(camera)
    except KeyboardInterrupt:
        pass
    finally:
        camera.shutdown()
        camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()