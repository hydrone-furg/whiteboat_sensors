#!/usr/bin/env python3

import rospy
from whiteboat_core.srv import RequestDataStream, RequestDataStreamResponse
from whiteboat_tools.common import MAVLinkConnection

mav_connection = None

def handle_data_stream_request(req):
    global mav_connection
    stream_type = req.stream_type

    if mav_connection is None or not mav_connection.connected:
        err_msg = "Erro: A conexão MAVLink não está ativa no nó do servidor."
        rospy.logerr(err_msg)
        return RequestDataStreamResponse(success=False, message=err_msg)
    
    success, msg = mav_connection.request_stream(stream_type)
    return RequestDataStreamResponse(success=success, message=msg)

def data_stream_server():
    global mav_connection
    rospy.init_node('data_stream_server')

    try:
        mav_connection = MAVLinkConnection(sitl_address='udp:127.0.0.1:14550', simulating=True)
        # mav_connection = MAVLinkConnection(baud=57600, simulating=False)
        mav_connection.connect()
    except Exception as e:
        rospy.logerr(e)

    service = rospy.Service('request_data_stream', RequestDataStream, handle_data_stream_request)
    rospy.loginfo("Serviço 'request_data_stream' pronto para receber chamadas.")
    rospy.spin()

if __name__ == "__main__":
    data_stream_server()
