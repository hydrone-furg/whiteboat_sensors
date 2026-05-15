#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from mavros_msgs.msg import State


class CameraSimNode(Node):
    def __init__(self):
        super().__init__('camera_sim_node')

        # Tópico padrão do VRX
        self.SIM_CAMERA_TOPIC = '/wamv/sensors/cameras/front_left_camera/image_raw'

        # Tópicos do Whiteboat
        self.CAMERA_TOPIC = '/whiteboat/sensors/camera/image_raw'
        self.MAVROS_STATE_TOPIC = '/whiteboat/state'

        self.declare_parameter('passthrough', True)
        self.declare_parameter('draw_overlay', False)
        self.declare_parameter('publish_only_when_subscribed', True)
        self.declare_parameter('max_fps', 10.0)
        self.PASSTHROUGH = bool(self.get_parameter('passthrough').value)
        self.DRAW_OVERLAY = bool(self.get_parameter('draw_overlay').value)
        self.PUBLISH_ONLY_WHEN_SUBSCRIBED = bool(self.get_parameter('publish_only_when_subscribed').value)
        self.MAX_FPS = max(0.0, float(self.get_parameter('max_fps').value))
        self._min_period_ns = int(1e9 / self.MAX_FPS) if self.MAX_FPS > 0.0 else 0
        self._last_pub_ns = 0

        # Inicializa bridge e estado
        self.bridge = CvBridge()
        self.mavros_connected = False
        self.current_mode = 'UNKNOWN'  # Inicializado antes do uso no callback

        from rclpy.qos import qos_profile_sensor_data

        # Publisher
        self.image_pub = self.create_publisher(Image, self.CAMERA_TOPIC, qos_profile_sensor_data)

        # Subscribers
        self.sim_image_sub = self.create_subscription(
            Image, self.SIM_CAMERA_TOPIC, self.sim_img_callback, qos_profile_sensor_data
        )
        self.mavros_state_sub = self.create_subscription(
            State, self.MAVROS_STATE_TOPIC, self.state_callback, 10
        )

        self._mavros_logged = False
        self._pub_logged = False
        self._last_logged_mode = None

        self.get_logger().info(
            f'Nó de Câmera Sim iniciado: passthrough={self.PASSTHROUGH}, '
            f'overlay={self.DRAW_OVERLAY}, max_fps={self.MAX_FPS:.1f}.'
        )

    def state_callback(self, msg: State):
        # Debug do estado MAVROS
        self.mavros_connected = msg.connected
        self.current_mode = msg.mode
        if not self._mavros_logged or msg.mode != self._last_logged_mode:
            self.get_logger().info(f'MAVROS Connected: {self.mavros_connected}')
            self.get_logger().info(f'MAVROS Mode: {self.current_mode}')
            self._mavros_logged = True
            self._last_logged_mode = msg.mode

    def sim_img_callback(self, msg: Image):
        if self.PUBLISH_ONLY_WHEN_SUBSCRIBED and self.image_pub.get_subscription_count() == 0:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._min_period_ns > 0 and now_ns - self._last_pub_ns < self._min_period_ns:
            return
        self._last_pub_ns = now_ns

        if self.PASSTHROUGH and not self.DRAW_OVERLAY:
            self.image_pub.publish(msg)
            if not self._pub_logged:
                self.get_logger().info('Imagem simulada repassada em modo leve.')
                self._pub_logged = True
            return

        try:
            # Converte ROS Image message para OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Erro ao converter imagem: {e}')
            return

        if self.DRAW_OVERLAY:
            (h, w) = cv_image.shape[:2]
            center = (w // 2, h // 2)
            cv2.circle(cv_image, center, 50, (0, 255, 0), 3)
            cv2.putText(
                cv_image,
                f'Mode: {self.current_mode}',
                (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2
            )

        # Debug local (descomente para ver a imagem)
        # cv2.imshow("Debug VRX", cv_image)
        # cv2.waitKey(1)

        # Publica imagem processada
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            processed_msg.header = msg.header
            self.image_pub.publish(processed_msg)

            if not self._pub_logged:
                self.get_logger().info('Imagem simulada publicada.')
                self._pub_logged = True

        except CvBridgeError as e:
            self.get_logger().error(f'Erro ao converter de volta para ROS: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
