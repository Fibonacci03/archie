#!/usr/bin/env python3
# Port de motor_classes.py (ROS1) al entorno ROS2.
# Sin dependencias de rospy — se usa directamente desde dynamixel_bridge.py.
from math import pi
from dynamixel_sdk import *


class Motor:
    def __init__(self, usb_port, dxl_baud_rate, list_ids, portHandler, packetHandler):
        self.portHandler = portHandler
        self.packetHandler = packetHandler
        self.protocol_version = 2.0
        self.list_ids = list_ids

    def communication(self, dxl_baud_rate, addr_torque_enable):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            raise RuntimeError("No se pudo abrir el puerto USB. Verifica la conexión del U2D2.")

        if self.portHandler.setBaudRate(dxl_baud_rate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            raise RuntimeError("No se pudo configurar el baudrate.")

        for id in self.list_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, id, addr_torque_enable, 0)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"[ID:{id:03d}] {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"[ID:{id:03d}] {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                print(f"[ID:{id:03d}] Dynamixel conectado")

    def torque(self, order, addr_torque_enable):
        for id in self.list_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, id, addr_torque_enable, order)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"{self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"{self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                state = "ON" if order == 1 else "OFF"
                print(f"Torque Motor {id}: {state}")

    def maxspeed(self, addr_max_velocity, value_max_velocity):
        for id in self.list_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                self.portHandler, id, addr_max_velocity, value_max_velocity)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"[ID:{id}] {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"[ID:{id}] {self.packetHandler.getRxPacketError(dxl_error)}")


class XCseries_motor(Motor):
    """
    Dynamixel 2XL430-W250-T
    https://emanual.robotis.com/docs/en/dxl/x/2xl430-w250/
    """
    def __init__(self, usb_port, dxl_baud_rate, list_ids, portHandler, packetHandler,
                 value_max_velocity, max_min_range_dict, list_pid):
        super().__init__(usb_port, dxl_baud_rate, list_ids, portHandler, packetHandler)

        self.addr_operating_mode     = 11
        self.addr_torque_enable      = 64
        self.addr_goal_position      = 116
        self.addr_present_position   = 132
        self.addr_present_velocity   = 128
        self.addr_goal_velocity      = 104
        self.torque_enable           = 1
        self.torque_disable          = 0
        self.addr_present_load       = 126
        self.minimum_position_value  = 0
        self.maximum_position_value  = 4095
        self.moving_status_threshold = 10
        self.addr_pos_pgain          = 84
        self.addr_pos_igain          = 82
        self.addr_pos_dgain          = 80
        self.max_angle_deg           = 360
        self.min_angle_deg           = 0
        self.addr_velocity_limit     = 44
        self.dict_range              = max_min_range_dict
        self.addr_goal_pwm           = 36
        # Perfil de movimiento (trapezoidal velocity profile del firmware Dynamixel)
        # Unidad Profile Velocity    : 0.229 rev/min  (0 = sin límite)
        # Unidad Profile Acceleration: 214.577 rev/min² (0 = sin límite)
        self.addr_profile_acceleration = 108
        self.addr_profile_velocity     = 112

        self.angle_zero = (self.maximum_position_value - self.minimum_position_value) // 2 + 1

        self.communication(dxl_baud_rate, self.addr_torque_enable)
        self.torque(self.torque_enable, self.addr_torque_enable)
        self.config_pid_cts(list_pid)

    def config_pid_cts(self, list_pid):
        addr_pid = [self.addr_pos_pgain, self.addr_pos_igain, self.addr_pos_dgain]
        for id in self.list_ids:
            for constant in range(len(list_pid[id])):
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
                    self.portHandler, id, addr_pid[constant], list_pid[id][constant])
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"{self.packetHandler.getTxRxResult(dxl_comm_result)}")
                elif dxl_error != 0:
                    print(f"{self.packetHandler.getRxPacketError(dxl_error)}")

    def set_profile(self, profile_velocity: int, profile_acceleration: int):
        """
        Configura el perfil de velocidad trapezoidal del firmware Dynamixel.

        profile_velocity    : velocidad máxima en unidades Dynamixel (1 unit = 0.229 rev/min).
                              0 = sin límite (movimiento brusco).
        profile_acceleration: aceleración en unidades Dynamixel (1 unit ≈ 214.577 rev/min²).
                              0 = sin límite (arranca al máximo).

        Valores de referencia:
          vel=30  → ~6.9 rpm  → suave y lento
          vel=100 → ~22.9 rpm → velocidad de trabajo normal
          acc=5   → rampa de aceleración gentil
          acc=15  → rampa de aceleración normal
        """
        for id in self.list_ids:
            for addr, val in [
                (self.addr_profile_acceleration, profile_acceleration),
                (self.addr_profile_velocity,     profile_velocity),
            ]:
                res, err = self.packetHandler.write4ByteTxRx(self.portHandler, id, addr, val)
                if res != COMM_SUCCESS:
                    print(f"[ID:{id}] set_profile error: {self.packetHandler.getTxRxResult(res)}")
                elif err != 0:
                    print(f"[ID:{id}] set_profile error: {self.packetHandler.getRxPacketError(err)}")

    def angleConversion(self, raw_value, to_radian_bool, id):
        """Convierte entre valor crudo (0-4095) y radianes. Idéntico al de motor_classes.py."""
        min_value_angle = self.minimum_position_value
        max_value_angle = self.maximum_position_value
        min_angle_deg   = self.min_angle_deg
        max_angle_deg   = self.max_angle_deg

        zero_value_angle       = (max_value_angle - min_value_angle) // 2 + 1  # 2048
        values_movement_span_2 = (max_value_angle - min_value_angle) // 2      # 2047
        deg_movement_span_2    = (max_angle_deg - min_angle_deg) // 2          # 180
        rad_movement_span_2    = float(deg_movement_span_2 * (pi / 180))       # π

        if to_radian_bool:
            if raw_value == zero_value_angle:
                return 0.0
            return (float(raw_value - zero_value_angle) / float(zero_value_angle)) * rad_movement_span_2
        else:
            if raw_value == 0.0:
                return zero_value_angle
            # Clamp dentro del rango articular
            raw_value = max(self.dict_range[id][0], min(self.dict_range[id][1], raw_value))
            return int(values_movement_span_2 * (raw_value / pi) + zero_value_angle)
