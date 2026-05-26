#!/usr/bin/env python3
"""
Nodo puente entre MoveIt2 y los motores Dynamixel 2XL430 de ARCHIE.

Expone exactamente los mismos tópicos/acciones que el arm_controller de simulación:
  - Acción servidor : arm_controller/follow_joint_trajectory  (FollowJointTrajectory)
  - Publisher       : /joint_states                          (sensor_msgs/JointState)

De esta forma, write_word.py, plan.py y MoveIt2 funcionan sin cambios tanto en
simulación como con el hardware real.
"""
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory

from dynamixel_sdk import (
    PortHandler, PacketHandler,
    GroupSyncWrite, GroupSyncRead,
    COMM_SUCCESS,
    DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD,
)

from .motor_driver import XCseries_motor

NUM_JOINTS  = 6
JOINT_NAMES = [f'joint_{i + 1}' for i in range(NUM_JOINTS)]

# PID y rangos articulares idénticos al nodo ROS1 (u2d2_communication.py)
_JOINT_RANGES = {
    0: [-1.57,   1.57],
    1: [-0.785,  0.785],
    2: [-1.15,   2.0],
    3: [-3.14,   3.14],
    4: [-1.15,   2.0],
    5: [-3.14,   3.14],
}
_PID_GAINS = {
    0: [1500,  450, 100],
    1: [4500, 1800, 800],
    2: [3500, 1500, 400],
    3: [ 300,    0,  30],
    4: [1500,  500, 150],
    5: [ 200,    0,  20],
}


class DynamixelBridge(Node):

    def __init__(self):
        super().__init__('archie_hardware_node')

        # Parámetros del nodo (mismos nombres que el launch de ROS1)
        self.declare_parameter('usb_port',      '/dev/ttyUSB0')
        self.declare_parameter('dxl_baud_rate', 1000000)
        # Perfil de movimiento Dynamixel (trapezoidal velocity profile)
        # profile_velocity    : 1 unit = 0.229 rev/min  (0 = sin límite → movimiento brusco)
        # profile_acceleration: 1 unit ≈ 214.577 rev/min² (0 = sin límite)
        self.declare_parameter('profile_velocity',     50)   # ~11.5 rpm — ajustar al gusto
        self.declare_parameter('profile_acceleration', 8)    # rampa suave
        # Si es True, al arrancar el nodo el robot se mueve suavemente a home (0 rad).
        self.declare_parameter('go_home_on_start', False)

        usb_port      = self.get_parameter('usb_port').value
        dxl_baud_rate = self.get_parameter('dxl_baud_rate').value
        self._profile_vel  = self.get_parameter('profile_velocity').value
        self._profile_acc  = self.get_parameter('profile_acceleration').value
        go_home_on_start   = self.get_parameter('go_home_on_start').value

        # Mutex: evita que el timer de lectura y el callback de trayectoria
        # accedan al hardware al mismo tiempo.
        self._hw_lock = threading.Lock()

        # Caché de posición actual (se actualiza en cada ciclo de lectura)
        self._current_positions = [0.0] * NUM_JOINTS

        # Inicializar hardware Dynamixel
        self._init_hardware(usb_port, dxl_baud_rate)

        # --- Grupos de callbacks ---
        # El timer de joint_states y el servidor de acción deben poder
        # ejecutarse en paralelo (MultiThreadedExecutor en main()).
        hw_cbg     = MutuallyExclusiveCallbackGroup()   # solo el timer
        action_cbg = ReentrantCallbackGroup()           # acción (puede quedar bloqueada esperando)

        # Publisher: /joint_states  ← mismo tópico que joint_state_broadcaster en sim
        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Servidor de acción: arm_controller/follow_joint_trajectory
        # ← mismo nombre que usa MoveIt2 según moveit_controllers.yaml
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            self._execute_trajectory,
            callback_group=action_cbg,
        )

        # Timer a 50 Hz para publicar estados articulares
        self.create_timer(0.02, self._publish_joint_states, callback_group=hw_cbg)

        # Servicio para mover el robot a la posición home (todos los joints a 0 rad)
        self.create_service(Trigger, 'go_home', self._go_home_callback)

        self.get_logger().info(
            f"ARCHIE hardware node listo  |  puerto: {usb_port}  |  baud: {dxl_baud_rate}  "
            f"|  profile_vel: {self._profile_vel}  |  profile_acc: {self._profile_acc}"
        )

        if go_home_on_start:
            self.get_logger().info('go_home_on_start=True → moviendo a home en 1 s...')
            self._home_timer = self.create_timer(1.0, self._home_once)

    # ------------------------------------------------------------------
    # Inicialización de hardware
    # ------------------------------------------------------------------

    def _init_hardware(self, usb_port: str, dxl_baud_rate: int):
        portHandler   = PortHandler(usb_port)
        packetHandler = PacketHandler(2.0)

        self._base = XCseries_motor(
            usb_port, dxl_baud_rate, [0, 1], portHandler, packetHandler,
            15,
            {0: _JOINT_RANGES[0], 1: _JOINT_RANGES[1]},
            {0: _PID_GAINS[0],    1: _PID_GAINS[1]},
        )
        self._codo = XCseries_motor(
            usb_port, dxl_baud_rate, [2, 3], portHandler, packetHandler,
            15,
            {2: _JOINT_RANGES[2], 3: _JOINT_RANGES[3]},
            {2: _PID_GAINS[2],    3: _PID_GAINS[3]},
        )
        self._ee = XCseries_motor(
            usb_port, dxl_baud_rate, [4, 5], portHandler, packetHandler,
            15,
            {4: _JOINT_RANGES[4], 5: _JOINT_RANGES[5]},
            {4: _PID_GAINS[4],    5: _PID_GAINS[5]},
        )

        self._motors      = [self._base, self._codo, self._ee]
        self._port        = portHandler
        self._packet      = packetHandler

        # GroupSyncWrite para enviar posiciones objetivo a todos los motores a la vez
        self._sync_write = GroupSyncWrite(
            portHandler, packetHandler, self._ee.addr_goal_position, 4)

        # GroupSyncRead para leer posiciones presentes de todos los motores a la vez
        self._sync_read = GroupSyncRead(
            portHandler, packetHandler, self._ee.addr_present_position, 4)

        for motor in self._motors:
            for mid in motor.list_ids:
                if not self._sync_read.addParam(mid):
                    self.get_logger().warn(f'[ID:{mid:03d}] groupSyncRead addParam falló')

        # Aplicar perfil de movimiento trapezoidal a todos los motores.
        # Esto limita la velocidad y aceleración máximas a nivel de firmware,
        # independientemente de lo que envíe el controlador de trayectoria.
        for motor in self._motors:
            motor.set_profile(self._profile_vel, self._profile_acc)
        self.get_logger().info(
            f'Perfil de movimiento aplicado  |  vel={self._profile_vel}  acc={self._profile_acc}'
        )

        # Leer posición actual de los motores para inicializar el caché.
        # NO se mueve el robot al arrancar — se respeta la posición física actual.
        initial = self._read_rad_positions()
        if initial is not None:
            self._current_positions = initial
            self.get_logger().info(
                f'Posiciones leídas al arrancar (rad): {[round(p, 3) for p in initial]}'
            )
        else:
            self.get_logger().warn(
                'No se pudo leer posición inicial. Se usará 0.0 rad para todos los joints.'
            )

    # ------------------------------------------------------------------
    # Escritura y lectura de hardware
    # ------------------------------------------------------------------

    def _send_rad_positions(self, positions_rad: list):
        """Convierte de radianes a raw y envía via SyncWrite."""
        raw = [0] * NUM_JOINTS
        for motor in self._motors:
            for mid in motor.list_ids:
                raw[mid] = motor.angleConversion(positions_rad[mid], False, mid)
        self._send_raw_positions(raw)

    def _send_raw_positions(self, raw_positions: list):
        """Envía valores crudos (0-4095) a todos los motores via SyncWrite."""
        for motor in self._motors:
            for mid in motor.list_ids:
                val = raw_positions[mid]
                param = [
                    DXL_LOBYTE(DXL_LOWORD(val)),
                    DXL_HIBYTE(DXL_LOWORD(val)),
                    DXL_LOBYTE(DXL_HIWORD(val)),
                    DXL_HIBYTE(DXL_HIWORD(val)),
                ]
                if not self._sync_write.addParam(mid, param):
                    self.get_logger().warn(f'[ID:{mid:03d}] SyncWrite addParam falló')

        result = self._sync_write.txPacket()
        if result != COMM_SUCCESS:
            self.get_logger().error(
                f'SyncWrite falló: {self._packet.getTxRxResult(result)}')
        self._sync_write.clearParam()

    def _read_rad_positions(self):
        """Lee posiciones presentes via SyncRead. Devuelve lista de radianes o None."""
        result = self._sync_read.txRxPacket()
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f'SyncRead falló: {self._packet.getTxRxResult(result)}')
            return None

        positions = [0.0] * NUM_JOINTS
        for motor in self._motors:
            for mid in motor.list_ids:
                if not self._sync_read.isAvailable(mid, motor.addr_present_position, 4):
                    self.get_logger().warn(f'[ID:{mid:03d}] SyncRead data no disponible')
                    continue
                raw = self._sync_read.getData(mid, motor.addr_present_position, 4)
                positions[mid] = motor.angleConversion(raw, True, mid)
        return positions

    # ------------------------------------------------------------------
    # Timer: publicar /joint_states
    # ------------------------------------------------------------------

    def _publish_joint_states(self):
        with self._hw_lock:
            positions = self._read_rad_positions()
        if positions is None:
            return

        self._current_positions = positions

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = JOINT_NAMES
        msg.position = positions
        self._js_pub.publish(msg)

    # ------------------------------------------------------------------
    # Acción: arm_controller/follow_joint_trajectory
    # ------------------------------------------------------------------

    # Período de interpolación: cada cuántos nanosegundos se envía un sub-punto.
    # 20 ms → 50 Hz, coincide con el timer de /joint_states.
    _INTERP_PERIOD_NS = 20_000_000

    def _execute_trajectory(self, goal_handle):
        """
        Ejecuta una trayectoria recibida de MoveIt2 con movimiento suave.

        Estrategia de suavizado (dos capas):
          1. Profile Velocity/Acceleration del firmware Dynamixel: limita velocidad
             y aceleración a nivel de hardware — actúa siempre, incluso entre sub-pasos.
          2. Interpolación lineal a 50 Hz: en lugar de enviar cada punto de golpe,
             se generan sub-pasos intermedios cada 20 ms.  Esto da al motor objetivos
             cercanos y frecuentes, reduciendo el "impulso" de posición.
        """
        traj        = goal_handle.request.trajectory
        joint_names = traj.joint_names
        points      = traj.points

        if not points:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        # Mapa: índice en la trayectoria → ID de motor (joint_X → ID X-1)
        traj_to_motor_id = {}
        for i, name in enumerate(joint_names):
            if name.startswith('joint_'):
                try:
                    motor_id = int(name.split('_')[1]) - 1
                    if 0 <= motor_id < NUM_JOINTS:
                        traj_to_motor_id[i] = motor_id
                except ValueError:
                    pass

        self.get_logger().info(
            f'Ejecutando trayectoria: {len(points)} puntos — interpolación a 50 Hz'
        )

        def extract_positions(point):
            """Extrae posiciones del punto de trayectoria en un array de NUM_JOINTS."""
            pos = list(self._current_positions)
            for traj_idx, motor_id in traj_to_motor_id.items():
                if traj_idx < len(point.positions):
                    pos[motor_id] = point.positions[traj_idx]
            return pos

        def point_time_ns(point):
            return point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec

        start_time = self.get_clock().now()

        # Punto de partida: posición actual del robot
        prev_pos = list(self._current_positions)
        prev_t   = 0  # nanosegundos desde start_time

        for point in points:
            curr_pos = extract_positions(point)
            curr_t   = point_time_ns(point)
            dt       = curr_t - prev_t

            if dt <= 0:
                # Punto sin avance temporal: enviar directamente y continuar
                with self._hw_lock:
                    self._send_rad_positions(curr_pos)
                prev_pos, prev_t = curr_pos, curr_t
                continue

            # Generar sub-pasos lineales cada _INTERP_PERIOD_NS
            n_steps = max(1, int(dt / self._INTERP_PERIOD_NS))

            for step in range(1, n_steps + 1):
                alpha = step / n_steps
                interp = [
                    p0 + alpha * (p1 - p0)
                    for p0, p1 in zip(prev_pos, curr_pos)
                ]

                # Tiempo objetivo de este sub-paso
                step_t = prev_t + int(step * dt / n_steps)

                # Esperar hasta ese instante
                while True:
                    elapsed = (self.get_clock().now() - start_time).nanoseconds
                    remaining = step_t - elapsed
                    if remaining <= 0:
                        break
                    time.sleep(min(remaining / 1e9, 0.002))

                # Chequear cancelación
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Trayectoria cancelada por el cliente.')
                    return FollowJointTrajectory.Result()

                with self._hw_lock:
                    self._send_rad_positions(interp)

            prev_pos, prev_t = curr_pos, curr_t

        goal_handle.succeed()
        self.get_logger().info('Trayectoria completada.')
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    # ------------------------------------------------------------------
    # Posición home
    # ------------------------------------------------------------------

    _HOME_POSITIONS = [0.0] * NUM_JOINTS   # todos los joints a 0 rad

    def _go_home_smooth(self):
        """
        Mueve el robot desde la posición actual hasta home usando la misma
        interpolación lineal a 50 Hz que _execute_trajectory.  El tiempo de
        movimiento se calcula para que ningún joint supere 0.5 rad/s.
        """
        with self._hw_lock:
            start = list(self._current_positions)

        max_delta = max(abs(h - s) for h, s in zip(self._HOME_POSITIONS, start))
        if max_delta < 0.01:
            self.get_logger().info('Ya está en home, no es necesario moverse.')
            return

        # Duración estimada: a lo más 0.5 rad/s (ajustar si se desea más lento/rápido)
        max_speed_rad_s = 0.5
        duration_s = max(max_delta / max_speed_rad_s, 1.0)   # mínimo 1 s
        n_steps = max(1, int(duration_s / (self._INTERP_PERIOD_NS / 1e9)))

        self.get_logger().info(
            f'Yendo a home en {duration_s:.1f} s ({n_steps} pasos)...'
        )

        for step in range(1, n_steps + 1):
            alpha = step / n_steps
            interp = [s + alpha * (h - s) for s, h in zip(start, self._HOME_POSITIONS)]
            with self._hw_lock:
                self._send_rad_positions(interp)
            time.sleep(self._INTERP_PERIOD_NS / 1e9)

        self._current_positions = list(self._HOME_POSITIONS)
        self.get_logger().info('Home alcanzado.')

    def _go_home_callback(self, request, response):
        """Callback del servicio /go_home (std_srvs/Trigger)."""
        try:
            self._go_home_smooth()
            response.success = True
            response.message = 'Home alcanzado correctamente.'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _home_once(self):
        """Timer de un disparo para ir a home al arrancar. Se auto-cancela."""
        self._home_timer.cancel()
        self._go_home_smooth()

    # ------------------------------------------------------------------
    # Apagado seguro
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Apagando torque de los motores...')
        with self._hw_lock:
            for motor in self._motors:
                motor.torque(motor.torque_disable, motor.addr_torque_enable)
        self._port.closePort()
        super().destroy_node()


# ----------------------------------------------------------------------
# Entrypoint
# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
