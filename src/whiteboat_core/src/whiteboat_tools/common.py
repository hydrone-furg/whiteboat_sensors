#!/usr/bin/env python3
from pymavlink import mavutil
import serial.tools.list_ports
# from whiteboat_tools.log import process_message, save_log_file
from time import sleep, time
import sys

class MAVLinkConnection:
    def __init__(self, baud=57600, sitl_address=0, simulating=False):
        self.port = None 
        pixhawk_port = find_pixhawk_port()

        if pixhawk_port and not simulating:
            print(f"Porta da Pixhawk detectada: {pixhawk_port}")
            self.port = pixhawk_port
        
        if simulating:
            print(f"Usando endereço SITL: {sitl_address}")
            self.port = sitl_address
        
        if self.port is None:
            raise ValueError("Nenhuma porta de conexão foi definida (Pixhawk ou SITL).")

        self.baud = baud
        self.connection = None
        self.target_system = None
        self.target_component = None
        self.logging = True
        self.connected = False

    def connect(self):
        try:
            print(f"Conectando a {self.port} com baudrate {self.baud}...") 
            self.connection = mavutil.mavlink_connection(self.port, baud=self.baud) # self.connection = mavutil.mavlink_connection(self.device)
            print("Aguardando heartbeat...")           
            self.connection.wait_heartbeat()
            print(f"Heartbeat recebido! (Sistema: {self.connection.target_system}, Componente: {self.connection.target_component})")
            self.target_system = self.connection.target_system
            self.target_component = self.connection.target_component
            self.connected = True
            
            # Habilitar streams importantes (opcional - algumas mensagens são enviadas por padrão)
            # streams = [
            #     mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            #     mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            #     mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            #     mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            #     mavutil.mavlink.MAV_DATA_STREAM_EXTRA2
            # ]
            # for stream_id in streams:
            #     self.connection.mav.request_data_stream_send(
            #         self.connection.target_system,
            #         self.connection.target_component,
            #         stream_id,
            #         10,  # 10 Hz
            #         1    # 1 para habilitar
            #     )
            
            sleep(1)
            return f"Conectado a {self.port}"
        except Exception as e:
            raise ConnectionError(f"Erro na conexão: {e}")

    def read_sensors(self):
        """
        Reads given sensor messages.

        Returns:
            MAVLink_message: Sensor message. https://github.com/PenguPilot/pymavlink/blob/master/mavlink.py
        """

        types = ['ATTITUDE', 'GPS_RAW_INT', 'SYS_STATUS', 'RAW_IMU', 'SCALED_IMU2', 'HIGHRES_IMU']

        msg = self.connection.recv_match(type=types, blocking=True, timeout=.1)
        return msg
    
    '''
    def record_log(self):
        try:
            logs = []
            types = ['ATTITUDE', 'GPS_RAW_INT', 'SYS_STATUS']

            while True:
                msg = self.connection.recv_match(type=types, blocking=True, timeout=1.0)

                if not msg:
                    print("Nenhuma mensagem recebida por 1 segundo...")
                else:
                    log_entry = process_message(msg)
                    logs.append(log_entry)
                    sleep(.05)

        except KeyboardInterrupt:
            save_log_file(logs)
            self.connection.close()
            return "Leitura interrompida pelo usuário"
    '''

    def request_message_interval(self, message_id: int, frequency_hz: float):
        """
        Request MAVLink message in a desired frequency,
        documentation for SET_MESSAGE_INTERVAL:
            https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

        Args:
            message_id (int): MAVLink message ID
            frequency_hz (float): Desired frequency in Hz
        """
        self.connection.mav.command_long_send(
            self.connection.target_system, self.connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, # The MAVLink message ID
            1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0, 0, 0, 0, # Unused parameters
            0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
        )

    def arm(self):
        """
        Arming the vehicle.
        """
        if not self.connected:
            return
        self.connection.mav.command_long_send(
            self.target_system, 
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, # 1 to arming
            0, 0, 0, 0, 0, 0
        )
        self.connection.motors_armed_wait()

    def disarm(self):
        """
        Disarming the vehicle.
        """
        if not self.connected:
            return
        self.connection.mav.command_long_send(
            self.target_system, 
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, # 0 to disarming
            0, 0, 0, 0, 0, 0
        )
        self.connection.motors_disarmed_wait()

    def set_rc_channel_pwm(self, channel_id: int, pwm: int = 1500):
        """
        Sets the PWM for the given RC channel.
        """
        if not self.connected:
            return
        if not 1 <= channel_id <= 18:
            print("Error: Invalid ID.")
            return
        rc_channel_values = [65535] * 18
        rc_channel_values[channel_id - 1] = pwm
        self.connection.mav.rc_channels_override_send(
            self.target_system, 
            self.target_component, 
            *rc_channel_values
        )
        
    def clear_rc_override(self):
        """
        Releases all RC channels, returning to manual control.
        """
        if not self.connected:
            return
        self.connection.mav.rc_channels_override_send(
            self.target_system,
            self.target_component,
            0, 0, 0, 0, 0, 0, 0, 0)
        print("RC override cleared.")
        
    def request_stream(self, stream_type: str):
        """
        Solicita a ativação de um determinado tipo de stream via MAVLink.

        Args:
            stream_type (str): tipo de stream (ex: "ATTITUDE", "GPS_RAW_INT", "SYS_STATUS")

        Returns:
            (bool, str): sucesso, mensagem
        """
        try:
            #conn = MAVLinkConnection(sitl_address="udp:127.0.0.1:14550", simulating=True)
            if not self.connected:
                return
            
            # Mapeamento de stream_type para MAVLink message IDs
            message_ids = {
                "ATTITUDE": 30,
                "GPS_RAW_INT": 24,
                "SYS_STATUS": 1,
                "RAW_IMU": 27,
                "SCALED_IMU2": 116,
                "HIGHRES_IMU": 105
            }

            if stream_type not in message_ids:
                return False, f"Tipo de stream '{stream_type}' não reconhecido."

            msg_id = message_ids[stream_type]
            self.connect().request_message_interval(msg_id, 10.0)  # 10 Hz
            return True, f"Solicitado stream '{stream_type}' com sucesso."

        except Exception as e:
            return False, f"Erro ao solicitar stream: {e}"

def find_pixhawk_port():
    # Lista todas as portas seriais disponíveis
    ports = serial.tools.list_ports.comports()
    
    # Caso 1: Verifica por portas ACM (USB comum para Pixhawk)
    for port in ports:
        if 'ACM' in port.device:
            print(f"Possível Pixhawk encontrado em {port.device} (USB/ACM)")
            return port.device
    
    # Caso 2: Verifica por portas USB (adaptadores FTDI/telemetria)
    for port in ports:
        if 'USB' in port.device:
            print(f"Possível Pixhawk encontrado em {port.device} (USB/FTDI)")
            return port.device
    
    # Caso 3: Verifica via descrição do hardware (vendor ID)
    for port in ports:
        if '3D Robotics' in port.description or 'Silicon Labs' in port.description or \
            'Pixhawk' in port.description:
            print(f"Pixhawk identificado por descrição em {port.device}")
            return port.device
    
    return None


