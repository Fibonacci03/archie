#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from std_srvs.srv import Empty
import copy
import math

class PlanNode(Node):
    def __init__(self):
        super().__init__('planing_node')
        
        self.cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.clear_tracer_client = self.create_client(Empty, 'clear_trace')
        
        # --- TUS VARIABLES GLOBALES AHORA SON ATRIBUTOS ---
        self.pen = 0.135
        self.t = 0.01
        self.y_h = 0.23
        self.size = 0.04
        self.space = 0.4 * self.size

        self.get_logger().info("Nodo de Planificación de Figuras listo.")

    # =========================================================
    # 1. HERRAMIENTAS BASE
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
        wpose.position.y = (self.y_h if d_y == self.y_h else (wpose.position.y + d_y))
        if d_z != 0:
            wpose.position.z = d_z
        waypoints.append(copy.deepcopy(wpose))
        return wpose, waypoints

    def set_pen(self, wpose, waypoints: list, p_x: float, p_y: float, p_z: float = 0):
        wpose.position.x = p_x
        wpose.position.y = p_y
        wpose.position.z = p_z
        waypoints.append(copy.deepcopy(wpose))
        return wpose, waypoints

    # =========================================================
    # 2. FUNCIONES DE FIGURAS (Pega el resto aquí)
    # =========================================================
    
    # Cuadrado
    def square(self, wpose, waypoints: list):
        # NOTA: Usar self.size, self.y_h, self.pen, self.move_pen, etc.
        wpose, waypoints = self.set_pen(wpose, waypoints, -self.size/2, self.y_h, self.pen + 0.02)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -self.size)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.size, 0)
        wpose, waypoints = self.up_pen(wpose, waypoints)
        wpose, waypoints = self.set_pen(wpose, waypoints, -self.size/2, self.y_h, self.pen + 0.02)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, self.size, self.y_h)
        wpose, waypoints = self.pen_up_down(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, 0, -self.size)
        wpose, waypoints = self.up_pen(wpose, waypoints)

        return waypoints, wpose

    # Triángulo
    def triangle(self, wpose, waypoints: list):

        side = self.size*2.5
        self.get_logger().info("Dibujando un triángulo...")
        wpose, waypoints = self.set_pen(wpose, waypoints, 0.0, self.y_h, self.pen + 0.02)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        wpose, waypoints = self.move_pen(wpose, waypoints, -side*math.cos(math.pi/3), -side*math.sin(math.pi/3))
        wpose, waypoints = self.move_pen(wpose, waypoints, side, 0.0)
        wpose, waypoints = self.move_pen(wpose, waypoints, -side*math.cos(math.pi/3), side*math.sin(math.pi/3))          
        wpose, waypoints = self.up_pen(wpose, waypoints)

        return waypoints, wpose
    
    # Circulo
    def circle(self, wpose, waypoints: list):
        # Parámetros base para un círculo de prueba
        center_x = wpose.position.x
        center_y = wpose.position.y + self.size
        r = self.size
        
        self.get_logger().info(f"Dibujando un círculo en X:{center_x} Y:{center_y}...")
        
        # Usamos un paso flotante para evitar errores de aserción
        for theta_deg in range(0, 361, 5):
            theta_rad = math.radians(float(theta_deg))
            wpose.position.x = center_x + r * math.cos(theta_rad)
            wpose.position.y = center_y + r * math.sin(theta_rad)
            # Forzamos que Z sea siempre flotante
            wpose.position.z = float(self.pen)
            
            waypoints.append(copy.deepcopy(wpose))

        return waypoints, wpose

    # =========================================================
    # 3. EL "MENÚ" INTELIGENTE
    # =========================================================

    def draw_figure(self, figure_name: str, start_pose: Pose):
        if self.clear_tracer_client.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            self.clear_tracer_client.call_async(req)
            self.get_logger().info("Pizarrón limpio.")
        else:
            self.get_logger().warn("Servicio de limpieza no encontrado, dibujando sobre lo anterior.")
        
        waypoints = [copy.deepcopy(start_pose)]
        wpose = copy.deepcopy(start_pose)
        
        self.get_logger().info(f"Preparando trazado para: '{figure_name}'")

        # MAGIA DE PYTHON: Busca la función por su nombre exacto
        if hasattr(self, figure_name):
            figure_function = getattr(self, figure_name)
            waypoints, wpose = figure_function(wpose, waypoints)
            self.execute_drawing(waypoints)
        else:
            self.get_logger().warn(f"Error: La figura '{figure_name}' no existe en la clase.")

    # =========================================================
    # 4. EJECUCIÓN CON MOVEIT 2
    # =========================================================

    def execute_drawing(self, waypoints):
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio compute_cartesian_path...')

        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.group_name = 'arm'
        req.waypoints = waypoints
        req.max_step = self.t
        req.jump_threshold = 0.0

        future = self.cartesian_client.call_async(req)
        future.add_done_callback(self.execute_path_callback)

    def execute_path_callback(self, future):
        try:
            response = future.result()
            if response.fraction > 0.9:
                goal_msg = ExecuteTrajectory.Goal()
            
                # Seteamos el tiempo a 0 para que el controlador lo acepte de inmediato
                # en lugar de comparar relojes de simulación.
                response.solution.joint_trajectory.header.stamp.sec = 0
                response.solution.joint_trajectory.header.stamp.nanosec = 0
                goal_msg.trajectory = response.solution
                self.get_logger().info("Enviando trayectoria al ejecutor...")
                self.execute_client.send_goal_async(goal_msg)
            else:
                self.get_logger().warn(f"Plan incompleto ({response.fraction*100}%).")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PlanNode()

    # Definir la posición de inicio
    start_pose = Pose()
    start_pose.position.x = 0.0
    start_pose.position.y = 0.2
    start_pose.position.z = node.pen + 0.00
    start_pose.orientation.x = 0.0
    start_pose.orientation.y = 1.0  # Rotación de 180 grados en Y
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 0.0

    # Llamamos a nuestro menú inteligente
    # Cambia "square" por "triangle", "circle", etc.
    node.draw_figure("triangle", start_pose)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()