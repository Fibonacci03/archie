#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, Trigger
import copy

class WriteWordNode(Node):
    def __init__(self):
        super().__init__('write_word_node')

        # Estado articular más reciente — se usa como start_state en compute_cartesian_path
        self._latest_joint_state = None
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # Clientes de ROS 2 para MoveIt
        self.cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Servidor 'execute_trajectory' no disponible. ¿Lanzaste MoveIt?")
        self.clear_tracer_client = self.create_client(Empty, 'clear_trace')
        self.go_home_client = self.create_client(Trigger, 'go_home')

        # --- TUS VARIABLES GLOBALES AHORA SON ATRIBUTOS ---
        self.pen = 0.135
        self.t = 0.01
        self.y_h = 0.23
        self.size = 0.04
        self.space = 0.4 * self.size

        self.get_logger().info("Nodo WriteWord listo para escribir.")

    def _js_cb(self, msg: JointState):
        if msg.position:
            self._latest_joint_state = msg

    # =========================================================
    # 1. HERRAMIENTAS DE DIBUJO BASE
    # =========================================================
    
    def pen_up_down(self, wpose, waypoints: list):
        wpose.position.z = self.pen + 0.05
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z = self.pen
        waypoints.append(copy.deepcopy(wpose))
        return wpose, waypoints

    def up_pen(self, wpose, waypoints: list):
        wpose.position.z = self.pen + 0.05
        waypoints.append(copy.deepcopy(wpose))
        return wpose, waypoints

    def down_pen(self, wpose, waypoints: list):
        wpose.position.z = self.pen
        waypoints.append(copy.deepcopy(wpose))
        return wpose, waypoints

    def move_pen(self, wpose, waypoints: list, d_x: float, d_y: float, d_z: float = 0):
        wpose.position.x += d_x
        if d_y == self.y_h:
            wpose.position.y = self.baseline_y
        else:
            wpose.position.y += d_y
            
        if d_z != 0:
            wpose.position.z = d_z
        waypoints.append(copy.deepcopy(wpose))
        return wpose, waypoints

    # =========================================================
    # 2. FUNCIONES DE LETRAS (Añade el resto de tu abecedario aquí)
    # =========================================================

    def plan_A(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -self.size)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, 0.5 * self.size)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, -self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.size + self.space, self.y_h)
        return waypoints, wpose

    def world_cup(self, wpose, waypoints: list):
        self.get_logger().info("Calculando trayectoria para la Copa del Mundo en write_word_node...")
        
        s = self.size * 1.5 # Factor de escala
        center_x = wpose.position.x
        start_y = self.y_h
        
        # Posicionarse en la esquina inferior izquierda
        wpose, waypoints = self.set_pen(wpose, waypoints, center_x - 0.3*s, start_y, self.pen + 0.05)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        
        # 1. Base
        wpose, waypoints = self.set_pen(wpose, waypoints, center_x + 0.3*s, start_y, self.pen)
        # 2. Escalón base derecho
        wpose, waypoints = self.set_pen(wpose, waypoints, center_x + 0.2*s, start_y + 0.2*s, self.pen)
        # 3. Soporte derecho
        wpose, waypoints = self.set_pen(wpose, waypoints, center_x + 0.4*s, start_y + 0.8*s, self.pen)
        wpose, waypoints = self.set_pen(wpose, waypoints, center_x + 0.5*s, start_y + 1.4*s, self.pen)
        
        # 4. Esfera del Mundo
        globe_center_y = start_y + 1.5*s
        globe_radius = 0.5*s
        import math
        for theta_deg in range(0, 181, 15):
            theta_rad = math.radians(theta_deg)
            gx = center_x + globe_radius * math.cos(theta_rad)
            gy = globe_center_y + globe_radius * math.sin(theta_rad)
            wpose, waypoints = self.set_pen(wpose, waypoints, gx, gy, self.pen)
            
        # 5. Soporte izquierdo
        wpose, waypoints = self.set_pen(wpose, waypoints, center_x - 0.4*s, start_y + 0.8*s, self.pen)
        wpose, waypoints = self.set_pen(wpose, waypoints, center_x - 0.2*s, start_y + 0.2*s, self.pen)
        # 6. Cerrar silueta
        wpose, waypoints = self.set_pen(wpose, waypoints, center_x - 0.3*s, start_y, self.pen)
        
        wpose, waypoints = self.up_pen(wpose, waypoints)
        
        return waypoints, wpose
    
    def plan_B(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.size * 0.85, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.size * 0.15, -self.size * 0.15)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -self.size * 0.2)
        wpose, waypoints = self.move_pen(wpose, waypoints, -self.size * 0.15, -self.size * 0.15)
        wpose, waypoints = self.move_pen(wpose, waypoints, -self.size * 0.85, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.size * 0.85, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.size * 0.15, -self.size * 0.15)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -self.size * 0.2)
        wpose, waypoints = self.move_pen(wpose, waypoints, -self.size * 0.15, -self.size * 0.15)
        wpose, waypoints = self.move_pen(wpose, waypoints, -self.size * 0.85, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, self.y_h)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.size + self.space, 0)
        return waypoints, wpose
    
    def plan_C(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h)
        return waypoints, wpose


    def plan_D(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size*0.85, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size*0.15,-self.size*0.15)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size*0.7)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size*0.15,-self.size*0.15)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size*0.85, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, self.y_h)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size + self.space, 0)
        return waypoints, wpose

    def plan_E(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,self.size*0.5)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size + self.space, self.y_h)
        return waypoints, wpose

    def plan_F(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,self.size*0.5)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h)
        return waypoints, wpose

    def plan_G(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)        
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size*0.85, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size*0.15,-self.size*0.15)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size*0.7)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size*0.15,-self.size*0.15)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size*0.7, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size*0.15,self.size*0.15)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,self.size*0.3)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size*0.3, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size*0.3 + self.space, self.y_h)
        return waypoints, wpose

    def plan_H(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,self.size/2)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,self.size/2)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h)
        return waypoints, wpose

    def plan_I(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size*0.5, 0)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size*0.5, 0)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h)
        return waypoints, wpose

    def plan_J(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size*0.35, 0)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size*0.5, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size*0.15,self.size*0.15)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size + self.space, self.y_h, self.pen + 0.02)

        return waypoints, wpose


    def plan_K(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, 0.5*self.size)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, -0.5*self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0.5*self.size)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, self.y_h)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h)

        return waypoints, wpose

    def plan_L(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h)

        return waypoints, wpose

    def plan_M(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.5*self.size, -0.5*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.5*self.size, 0.5*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h)

        return waypoints, wpose

    def plan_N(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size,-self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, self.y_h)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h)

        return waypoints, wpose

    def plan_O(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size + self.space, self.y_h)    

        return waypoints, wpose

    def plan_P(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -0.5*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size + self.space, self.y_h)

        return waypoints, wpose

    def plan_Q(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.5*self.size, -0.5*self.size)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.5*self.size, -0.5*self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h)

        return waypoints, wpose

    def plan_R(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -0.5*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, -0.5*self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h)

        return waypoints, wpose

    def plan_S(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size*0.5)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size*0.5)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size + self.space, self.y_h)    
        
        return waypoints, wpose

    def plan_T(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)   
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, -0.5*self.size, 0)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.5*self.size + self.space, self.y_h)

        return waypoints, wpose

    def plan_U(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)    
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, self.y_h)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h)

        return waypoints, wpose

    def plan_V(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints) 
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.5*self.size,-self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.5*self.size, self.y_h)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, 0)

        return waypoints, wpose

    def plan_W(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)  
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.5*self.size, 0.5*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.5*self.size, -0.5*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, self.y_h)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, 0)

        return waypoints, wpose

    def plan_X(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size,-self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size,self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h, self.pen + 0.02)

        return waypoints, wpose

    def plan_Y(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size*0.5)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,self.size*0.5)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size + self.space, self.y_h, self.pen + 0.02)

        return waypoints, wpose

    def plan_Z(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size,-self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h, self.pen + 0.02)

        return waypoints, wpose\
        
    def plan_space(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints, self.space*5, self.y_h, self.pen + 0.02)
        return waypoints, wpose

    def plan_1(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -0.3*self.size)     
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.5*self.size, 0.3*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, -0.5*self.size, 0)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h, self.pen + 0.02)

        return waypoints, wpose

    def plan_2(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -0.5*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -0.5*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h, self.pen + 0.02)

        return waypoints, wpose

    def plan_3(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.2*self.size, 0.5*self.size)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.8*self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h, self.pen + 0.02)

        return waypoints, wpose

    def plan_4(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -0.5*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, 0.5*self.size)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h, self.pen + 0.02)

        return waypoints, wpose

    def plan_5(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -0.5*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.8*self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.2*self.size, -0.2*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -0.1*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, -0.2*self.size, -0.2*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, -0.8*self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size + self.space, self.y_h, self.pen + 0.02)

        return waypoints, wpose

    def plan_6(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,-self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, 0.5*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size + self.space, self.y_h, self.pen + 0.02)

        return waypoints, wpose

    def plan_7(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size,-self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size + self.space, self.y_h, self.pen + 0.02)

        return waypoints, wpose

    def plan_8(self, wpose, waypoints: list):

        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -0.5*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -0.5*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, 0.5*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,self.size*0.5)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size + self.space, self.y_h, self.pen + 0.02)

        return waypoints, wpose

    def plan_9(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints,self.size,-self.size)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0,self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,-self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -0.5*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints,self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.space, self.y_h, self.pen + 0.02)

        return waypoints, wpose

    def plan_0(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints, 0.15*self.size, 0)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.7*self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.15*self.size, -0.15*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -0.7*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, -0.15*self.size, -0.15*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, -0.7*self.size, 0)
        wpose, waypoints = self.move_pen(wpose, waypoints, -0.15*self.size, 0.15*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, 0.7*self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.15*self.size, 0.15*self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.85*self.size + self.space, self.y_h, self.pen + 0.02)

        return waypoints, wpose

    def plan_minus(self, wpose, waypoints: list):

        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -0.5*self.size)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0.5*self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 2.5*self.space, self.y_h, self.pen + 0.02)

        return waypoints, wpose

    # =========================================================
    # 3. LÓGICA DE PROCESAMIENTO
    # =========================================================

    def write_string(self, word: str, start_pose: Pose):
        # Borrar palabras escritas
        if self.clear_tracer_client.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            self.clear_tracer_client.call_async(req)
            self.get_logger().info("Pizarrón limpio.")
        else:
            self.get_logger().warn("Servicio de limpieza no encontrado, dibujando sobre lo anterior.")
        
        # Definir puntos iniciales
        self.baseline_y = start_pose.position.y
        waypoints = [copy.deepcopy(start_pose)]
        wpose = copy.deepcopy(start_pose)

        # Log de palabra ingresada 
        self.get_logger().info(f"Procesando palabra: '{word}'")
        
        #Escritura de palabra
        for char in word.upper():
            if char == " ":
                # Si es un espacio, solo movemos la pluma
                wpose, waypoints = self.move_pen(wpose, waypoints, self.size + self.space, 0)
                continue
            
            # Busca dinámicamente la función "plan_A", "plan_B", etc.
            func_name = f"plan_{char}"
            if hasattr(self, func_name):
                letter_function = getattr(self, func_name)
                waypoints, wpose = letter_function(wpose, waypoints)
            else:
                self.get_logger().warn(f"No existe una función para la letra: {char}")

        self.execute_drawing(waypoints)

    def execute_drawing(self, waypoints):
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio compute_cartesian_path...')

        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.group_name = 'arm'
        req.waypoints = waypoints
        req.max_step = self.t
        req.jump_threshold = 0.0

        # Pasar el estado articular actual explícitamente.
        # Sin esto MoveIt2 lo busca en su state monitor interno, que puede
        # no haberse actualizado aún → "Found empty JointState message".
        if self._latest_joint_state is not None:
            start_state = RobotState()
            start_state.joint_state = self._latest_joint_state
            req.start_state = start_state

        future = self.cartesian_client.call_async(req)
        future.add_done_callback(self.execute_path_callback)

    def execute_path_callback(self, future):
        try:
            response = future.result()
            if response.fraction > 0.9:
                goal_msg = ExecuteTrajectory.Goal()
                response.solution.joint_trajectory.header.stamp.sec = 0
                response.solution.joint_trajectory.header.stamp.nanosec = 0
                goal_msg.trajectory = response.solution
                self.get_logger().info("Enviando trayectoria al ejecutor...")
                send_future = self.execute_client.send_goal_async(goal_msg)
                send_future.add_done_callback(self._on_goal_accepted)
            else:
                self.get_logger().warn(f"Plan incompleto ({response.fraction*100}%).")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def _on_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trayectoria rechazada por el ejecutor.")
            return
        self.get_logger().info("Trayectoria aceptada, ejecutando...")
        goal_handle.get_result_async().add_done_callback(self._on_trajectory_done)

    def _on_trajectory_done(self, future):
        self.get_logger().info("Escritura completada. Volviendo a posición inicial...")
        if self.go_home_client.wait_for_service(timeout_sec=2.0):
            future_home = self.go_home_client.call_async(Trigger.Request())
            future_home.add_done_callback(
                lambda f: self.get_logger().info(
                    f"Home: {f.result().message}" if f.result() else "Home: sin respuesta"
                )
            )
        else:
            self.get_logger().warn("Servicio /go_home no disponible (¿modo simulación?).")

def main(args=None):
    rclpy.init(args=args)
    
    # Asegúrate de que este sea el nombre correcto de tu clase
    node = WriteWordNode() 

    # Definir la posición de inicio
    start_pose = Pose()
    start_pose.position.x = 0.0
    start_pose.position.y = 0.2
    start_pose.position.z = node.pen + 0.00
    start_pose.orientation.x = 0.0
    start_pose.orientation.y = 1.0  # Rotación de 180 grados en Y
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 0.0

    # --- RESTAURAMOS LA INSTRUCCIÓN ORIGINAL ---
    node.write_word('RIMP', start_pose)
    # -------------------------------------------

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
