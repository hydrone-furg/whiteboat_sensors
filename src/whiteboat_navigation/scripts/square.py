#!/usr/bin/env python3
"""
Navegação autônoma em quadrado para o WAM-V.
Faz o barco percorrer 4 lados de 6s com giros de 90°.
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
import math
import time
from sensor_msgs.msg import Imu
from mavros_msgs.msg import OverrideRCIn, State
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion


class SquareNode(Node):
    def __init__(self):
        super().__init__('square_node')

        # Configurações de Tópicos e Modos
        self.declare_parameter('rc_override_topic', '/whiteboat/rc/override')
        self.declare_parameter('imu_topic', '/whiteboat/imu/data')
        self.declare_parameter('imu_fallback_topics', [
            '/whiteboat/mavros/imu/data',
            '/mavros/imu/data',
            '/wamv/sensors/imu/imu/data',
            '/sensors/imu/imu/data'
        ])
        self.declare_parameter('require_mavros_mode', True)
        self.declare_parameter('mode_topic', '/whiteboat/state')
        self.declare_parameter('command_mode', 'rc_override') # 'rc_override' ou 'vrx_direct'
        self.declare_parameter('vrx_left_thrust_topic', '/wamv/thrusters/left/thrust')
        self.declare_parameter('vrx_right_thrust_topic', '/wamv/thrusters/right/thrust')
        self.declare_parameter('vrx_left_pos_topic', '/wamv/thrusters/left/pos')
        self.declare_parameter('vrx_right_pos_topic', '/wamv/thrusters/right/pos')
        self.declare_parameter('vrx_thrust_max', 5000.0)

        self.RC_OVERRIDE_TOPIC = self.get_parameter('rc_override_topic').get_parameter_value().string_value
        self.IMU_TOPIC = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.IMU_FALLBACK_TOPICS = [
            topic for topic in self.get_parameter('imu_fallback_topics').get_parameter_value().string_array_value
            if topic and topic != self.IMU_TOPIC
        ]
        self.REQUIRE_MAVROS_MODE = bool(self.get_parameter('require_mavros_mode').value)
        self.MODE_TOPIC = self.get_parameter('mode_topic').get_parameter_value().string_value
        self.COMMAND_MODE = self.get_parameter('command_mode').get_parameter_value().string_value
        self.VRX_LEFT_THRUST_TOPIC = self.get_parameter('vrx_left_thrust_topic').get_parameter_value().string_value
        self.VRX_RIGHT_THRUST_TOPIC = self.get_parameter('vrx_right_thrust_topic').get_parameter_value().string_value
        self.VRX_LEFT_POS_TOPIC = self.get_parameter('vrx_left_pos_topic').get_parameter_value().string_value
        self.VRX_RIGHT_POS_TOPIC = self.get_parameter('vrx_right_pos_topic').get_parameter_value().string_value
        self.VRX_THRUST_MAX = max(0.0, float(self.get_parameter('vrx_thrust_max').value))

        if self.COMMAND_MODE not in ('rc_override', 'vrx_direct'):
            raise ValueError(
                f"command_mode='{self.COMMAND_MODE}' inválido. "
                "Use 'rc_override' (via bridge, recomendado) ou 'vrx_direct' (sem MAVROS)."
            )

        # Constantes de navegação (valores suavizados)
        self.THRUST_NEUTRAL = 0.0
        self.THRUST_FORWARD = 35.0     # Potência máxima frente
        self.SIDE_DURATION = 6.0       # Duração do lado
        self.WAIT_DURATION = 3.0       # Espera antes do giro (deixar inércia dissipar)

        self.THRUST_TURN = 15.0        # Potência máxima giro (reduzido para evitar overshoot)
        self.THRUST_TURN_MIN = 5.0     # Potência mínima giro (perto do alvo)
        self.COAST_ANGLE = 25.0        # Graus: abaixo disso, para motores e deixa inércia agir
        self.DEGREE_RANGE = 8.0        # Tolerância angular (graus)
        self.RAMP_DURATION = 2.0       # Segundos para atingir potência máxima
        self._current_thrust = 0.0     # Thrust atual (para rampa)
        self._prev_error_sign = 0      # Para detectar overshoot

        # Faixa de PWM para RC Override (compatível com ArduPilot/MAVROS)
        self.RC_MIN = 1100
        self.RC_NEUTRAL = 1500
        self.RC_MAX = 1900
        self.THROTTLE_CHANNEL = 2   # SERVO3 → Throttle
        self.STEERING_CHANNEL = 0   # SERVO1 → Steering
        # ArduRover costuma usar MANUAL; STEERING/ACRO também permitem override RC.
        self._modes_rc_ok = frozenset({'MANUAL', 'STEERING', 'ACRO'})

        from rclpy.qos import qos_profile_sensor_data

        # Publishers
        self.rc_override_pub = self.create_publisher(OverrideRCIn, self.RC_OVERRIDE_TOPIC, 1)
        if self.COMMAND_MODE == 'vrx_direct':
            self.vrx_left_thrust_pub = self.create_publisher(Float64, self.VRX_LEFT_THRUST_TOPIC, 10)
            self.vrx_right_thrust_pub = self.create_publisher(Float64, self.VRX_RIGHT_THRUST_TOPIC, 10)
            self.vrx_left_pos_pub = self.create_publisher(Float64, self.VRX_LEFT_POS_TOPIC, 10)
            self.vrx_right_pos_pub = self.create_publisher(Float64, self.VRX_RIGHT_POS_TOPIC, 10)

        # Subscribers IMU (primário + fallback)
        self.imu_subscriptions = []
        for topic in [self.IMU_TOPIC] + self.IMU_FALLBACK_TOPICS:
            self.imu_subscriptions.append(
                self.create_subscription(
                    Imu,
                    topic,
                    lambda msg, source_topic=topic: self.imu_callback(msg, source_topic),
                    qos_profile_sensor_data,
                )
            )
        self.mode_sub = self.create_subscription(State, self.MODE_TOPIC, self.mode_callback, 10)

        self.get_logger().info(
            f'square_node: IMU={self.IMU_TOPIC} | MODE={self.MODE_TOPIC} | '
            f'RC_OVERRIDE={self.RC_OVERRIDE_TOPIC} | COMMAND_MODE={self.COMMAND_MODE}'
        )
        if self.COMMAND_MODE == 'vrx_direct':
            self.get_logger().info(
                f'VRX direto: left={self.VRX_LEFT_THRUST_TOPIC}, right={self.VRX_RIGHT_THRUST_TOPIC}, '
                f'thrust_max={self.VRX_THRUST_MAX:.1f}'
            )
        if self.IMU_FALLBACK_TOPICS:
            self.get_logger().info(f'IMU fallback ativo: {", ".join(self.IMU_FALLBACK_TOPICS)}')

        self.get_logger().info('Aguardando IMU...')
        self.current_yaw = None
        self.target_yaw = None
        self.active_imu_topic = None

        self.last_imu_time_s = None
        self.imu_timeout_s = 1.0

        self.state = 'WAITING_FOR_IMU'
        self.state_start_time = None
        self.side_counter = 0

        # Flags para simular loginfo_once
        self._logged_waiting_imu = False
        self._logged_done = False
        self._logged_imu_failure = False
        self._logged_incorrect_mode = False

        # Timer a 20 Hz usando STEADY clock (não afetado por use_sim_time)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / 20.0, self.run, clock=self.steady_clock)
        self._last_rc_sub_warn_s = 0.0
        self._last_vrx_sub_warn_s = 0.0
        self._last_imu_wait_log_s = 0.0
        self._last_mode_wait_log_s = 0.0
        self._shutting_down = False

    def mode_callback(self, data: State):
        if data.mode in self._modes_rc_ok:
            if self.state == 'WAITING_FOR_MODE':
                self.change_state('FORWARD')
        else:
            # Só aborta se já estivermos comandando movimento; ignorar modo FCU
            # durante WAITING_FOR_IMU / WAITING_FOR_MODE evita INCORRECT_MODE
            # prematuro (ex.: HOLD antes do modo RC adequado).
            if self.state in ('FORWARD', 'TURN', 'WAIT_TO_STOP'):
                self.change_state('INCORRECT_MODE')

    def imu_callback(self, msg: Imu, source_topic=None):
        self.last_imu_time_s = time.monotonic()
        if self.active_imu_topic is None and source_topic is not None:
            self.active_imu_topic = source_topic
            if source_topic != self.IMU_TOPIC:
                self.get_logger().warn(
                    f'IMU recebida em fallback {source_topic}; tópico primário configurado é {self.IMU_TOPIC}. '
                    'Para fixar, execute com --ros-args -p imu_topic:=' + source_topic
                )
        orientation_q = msg.orientation
        _, _, self.current_yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        if self.state == 'WAITING_FOR_IMU':
            self.get_logger().info('Dados da IMU recebidos!')
            self.change_state('WAITING_FOR_MODE')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def set_thrusters(self, left_thrust, right_thrust):
        """Envia comandos de força aos motores."""
        if not rclpy.ok():
            return

        if self.COMMAND_MODE == 'vrx_direct':
            self._publish_vrx_direct(left_thrust, right_thrust)
            return

        # Modo 'rc_override' — converte left/right → throttle/steering PWM
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

        # Avisa se ninguém está ouvindo o RC Override
        pushing = abs(left_thrust) + abs(right_thrust) > 1e-3
        if pushing:
            try:
                n_sub = self.rc_override_pub.get_subscription_count()
            except Exception:
                n_sub = -1
            if n_sub == 0:
                now_s = time.monotonic()
                if now_s - self._last_rc_sub_warn_s > 5.0:
                    self._last_rc_sub_warn_s = now_s
                    self.get_logger().warn(
                        f'Nenhum subscritor em {self.RC_OVERRIDE_TOPIC} (subscription_count=0). '
                        'O MAVROS e/ou a bridge não estão a receber este tópico — '
                        'verifica o nome com ros2 topic list.'
                    )

        try:
            if rclpy.ok():
                self.rc_override_pub.publish(rc_msg)
        except Exception:
            pass

    def _publish_vrx_direct(self, left_thrust, right_thrust):
        """Publica diretamente nos thrusters VRX (sem bridge/MAVROS).

        Usa mixagem diferencial com ângulo de posição dos thrusters para
        controle de direção, similar a como funciona a bridge.
        """
        # Normalizar thrust para a faixa do VRX
        left_norm = max(-1.0, min(1.0, left_thrust / self.THRUST_FORWARD))
        right_norm = max(-1.0, min(1.0, right_thrust / self.THRUST_FORWARD))

        left_msg = Float64(data=left_norm * self.VRX_THRUST_MAX)
        right_msg = Float64(data=right_norm * self.VRX_THRUST_MAX)

        # Posição dos thrusters: 0.0 rad = apontando para frente
        pos_msg = Float64(data=0.0)

        self.vrx_left_pos_pub.publish(pos_msg)
        self.vrx_right_pos_pub.publish(pos_msg)
        self.vrx_left_thrust_pub.publish(left_msg)
        self.vrx_right_thrust_pub.publish(right_msg)

        if abs(left_thrust) + abs(right_thrust) > 1e-3:
            vrx_subscribers = (
                self.vrx_left_thrust_pub.get_subscription_count() +
                self.vrx_right_thrust_pub.get_subscription_count()
            )
            if vrx_subscribers == 0:
                now_s = time.monotonic()
                if now_s - self._last_vrx_sub_warn_s > 5.0:
                    self._last_vrx_sub_warn_s = now_s
                    self.get_logger().warn(
                        f'Nenhum subscritor nos thrusters VRX '
                        f'({self.VRX_LEFT_THRUST_TOPIC}, {self.VRX_RIGHT_THRUST_TOPIC}). '
                        'Lance o VRX em sim_mode:=full/bridge com as bridges do WAM-V ativas.'
                    )

    def release_control(self):
        if not rclpy.ok():
            return
        if not getattr(self, '_logged_release', False):
            self.get_logger().info('Liberando controle RC...')
            self._logged_release = True

        rc_msg = OverrideRCIn()
        rc_msg.channels = [0] * 18
        try:
            if rclpy.ok():
                self.rc_override_pub.publish(rc_msg)
        except Exception:
            pass

    def stopping_the_boat(self):
        self.set_thrusters(0.0, 0.0)

    def shutdown(self):
        self._shutting_down = True
        if rclpy.ok():
            self.get_logger().info('Barco parado, desligando o nó.')
        self.stopping_the_boat()
        self.release_control()

    def change_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(f'Mudando de estado: {self.state} -> {new_state}')
            self.state = new_state

            if self.state == 'FORWARD':
                self.state_start_time = time.monotonic()
                self.get_logger().info(
                    f'#--- LADO {self.side_counter + 1}: Iniciando movimento em linha reta... ---#'
                )

            elif self.state == 'TURN':
                self.target_yaw = self.normalize_angle(self.current_yaw - math.radians(90))

            elif self.state == 'WAIT_TO_STOP':
                self.state_start_time = time.monotonic()
                self.get_logger().info(
                    f'#--- Aguardando {self.WAIT_DURATION} segundos para o barco parar... ---#'
                )

    def run(self):
        if self.state == 'WAITING_FOR_IMU':
            if not self._logged_waiting_imu:
                self.get_logger().info('Aguardando a primeira mensagem da IMU...')
                self._logged_waiting_imu = True
            now_w = time.monotonic()
            if now_w - self._last_imu_wait_log_s >= 4.0:
                self._last_imu_wait_log_s = now_w
                self.get_logger().warn(
                    'Ainda aguardando IMU. '
                    f'Assinando: {[self.IMU_TOPIC] + self.IMU_FALLBACK_TOPICS}.'
                )
            return

        if self.state == 'WAITING_FOR_MODE':
            if not self.REQUIRE_MAVROS_MODE:
                self.get_logger().warn(
                    'require_mavros_mode=false: iniciando percurso sem aguardar modo MAVROS.'
                )
                self.change_state('FORWARD')
                return

            now_w = time.monotonic()
            if now_w - self._last_mode_wait_log_s >= 4.0:
                self._last_mode_wait_log_s = now_w
                mode_publishers = len(self.get_publishers_info_by_topic(self.MODE_TOPIC))
                rc_subscribers = self.rc_override_pub.get_subscription_count()
                self.get_logger().warn(
                    f'Aguardando modo MAVROS em {self.MODE_TOPIC}. '
                    f'Publishers state={mode_publishers}; subscribers RC override={rc_subscribers}; '
                    f'modos aceitos={sorted(self._modes_rc_ok)}. '
                    'Se estiver em sim sem MAVROS ativo, rode com -p require_mavros_mode:=false.'
                )
            return

        now_s = time.monotonic()
        if self.last_imu_time_s is None or now_s - self.last_imu_time_s > self.imu_timeout_s:
            if not self._logged_imu_failure:
                self.get_logger().warn('Aviso: Dados da IMU atrasados! O simulador pode estar lento. Aguardando...')
                self._logged_imu_failure = True
            return
        else:
            if self._logged_imu_failure:
                self.get_logger().info('IMU reconectada e operante!')
                self._logged_imu_failure = False

        if self.state == 'FORWARD':
            # Rampa de aceleração suave
            elapsed = now_s - self.state_start_time if self.state_start_time else 0
            ramp = min(1.0, elapsed / self.RAMP_DURATION)
            thrust = self.THRUST_FORWARD * ramp
            self.set_thrusters(thrust, thrust)
            if self.state_start_time is None:
                raise Exception('ERRO: Faltando o start time!')
            elif now_s - self.state_start_time >= self.SIDE_DURATION:
                self.side_counter += 1
                if self.side_counter >= 4:
                    self.change_state('DONE')
                else:
                    self.change_state('WAIT_TO_STOP')

        elif self.state == 'TURN':
            error_rad = self.normalize_angle(self.target_yaw - self.current_yaw)
            error_degrees = math.degrees(error_rad)
            abs_error = abs(error_degrees)
            self.get_logger().info(
                f'Alvo: {math.degrees(self.target_yaw):.1f}, '
                f'Atual: {math.degrees(self.current_yaw):.1f}, '
                f'Erro: {error_degrees:.1f}°',
                throttle_duration_sec=1
            )
            if abs_error <= self.DEGREE_RANGE:
                # Chegou ao alvo — parar motores e avançar
                self.stopping_the_boat()
                self._prev_error_sign = 0
                self.change_state('FORWARD')
            elif abs_error <= self.COAST_ANGLE:
                # Zona de costa: perto do alvo, desliga motores e
                # deixa a inércia angular completar o giro
                self.stopping_the_boat()
                self.get_logger().info(
                    f'Coast zone ({abs_error:.1f}° restantes, motores OFF)',
                    throttle_duration_sec=1
                )
            else:
                # Zona ativa: thrust proporcional ao erro
                # Escala de 0→1 entre COAST_ANGLE e 90°
                error_ratio = min(1.0, (abs_error - self.COAST_ANGLE) / (90.0 - self.COAST_ANGLE))
                turn_thrust = self.THRUST_TURN_MIN + (self.THRUST_TURN - self.THRUST_TURN_MIN) * error_ratio
                if error_degrees < 0:
                    self.set_thrusters(-turn_thrust, turn_thrust)
                else:
                    self.set_thrusters(turn_thrust, -turn_thrust)
                self._prev_error_sign = 1 if error_degrees > 0 else -1

        elif self.state == 'WAIT_TO_STOP':
            self.stopping_the_boat()
            if self.state_start_time is None:
                raise Exception('ERRO: Faltando o start time!')
            elif now_s - self.state_start_time >= self.WAIT_DURATION:
                self.change_state('TURN')

        elif self.state == 'DONE':
            if not self._logged_done:
                self.get_logger().info('Percurso do quadrado finalizado!')
                self._logged_done = True
            self.stopping_the_boat()
            self.release_control()

        elif self.state == 'IMU_FAILURE':
            if not self._logged_imu_failure:
                self.get_logger().error('Falha da IMU!')
                self._logged_imu_failure = True
            self.shutdown()

        elif self.state == 'INCORRECT_MODE':
            if not self._logged_incorrect_mode:
                self.get_logger().warn('Modo de voo incorreto!')
                self._logged_incorrect_mode = True
            self.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SquareNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._shutting_down = True
        if rclpy.ok():
            node.get_logger().info('Parando o barco...')
        node.stopping_the_boat()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
