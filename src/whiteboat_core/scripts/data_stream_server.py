#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from whiteboat_core.srv import RequestDataStream
from whiteboat_tools.common import MAVLinkConnection


class DataStreamServer(Node):
    def __init__(self):
        super().__init__('data_stream_server')

        self.mav_connection = None

        try:
            self.mav_connection = MAVLinkConnection(
                sitl_address='udp:127.0.0.1:14550', simulating=True
            )
            # self.mav_connection = MAVLinkConnection(baud=57600, simulating=False)
            self.mav_connection.connect()
        except Exception as e:
            self.get_logger().error(str(e))

        self.srv = self.create_service(
            RequestDataStream,
            'request_data_stream',
            self.handle_data_stream_request
        )
        self.get_logger().info("Serviço 'request_data_stream' pronto para receber chamadas.")

    def handle_data_stream_request(self, request, response):
        stream_type = request.stream_type

        if self.mav_connection is None or not self.mav_connection.connected:
            err_msg = 'Erro: A conexão MAVLink não está ativa no nó do servidor.'
            self.get_logger().error(err_msg)
            response.success = False
            response.message = err_msg
            return response

        success, msg = self.mav_connection.request_stream(stream_type)
        response.success = success
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DataStreamServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
