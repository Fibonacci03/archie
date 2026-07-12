#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
import copy
import math

class PlanNode(Node):
    def __init__(self):
        super().__init__('planing_node')
        
        self.cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.clear_tracer_client = self.create_client(Empty, 'clear_trace')
        
        # --- PUBLICADOR PARA DIBUJAR LA TRAYECTORIA EN RVIZ ---
        self.marker_pub = self.create_publisher(Marker, 'trajectory_trace', 10)
        
        # --- VARIABLES GLOBALES ---
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
    # 2. FUNCIONES DE FIGURAS
    # =========================================================
    
    # Cuadrado
    def square(self, wpose, waypoints: list):
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
        center_x = wpose.position.x
        center_y = wpose.position.y + self.size
        r = self.size
        self.get_logger().info(f"Dibujando un círculo en X:{center_x} Y:{center_y}...")
        for theta_deg in range(0, 361, 5):
            theta_rad = math.radians(float(theta_deg))
            wpose.position.x = center_x + r * math.cos(theta_rad)
            wpose.position.y = center_y + r * math.sin(theta_rad)
            wpose.position.z = float(self.pen)
            waypoints.append(copy.deepcopy(wpose))
        return waypoints, wpose

    # Silueta de un Automóvil (SUV)
    def car_profile(self, wpose, waypoints: list):
        self.get_logger().info("Calculando trayectoria para la silueta de automóvil...")
        
        s = self.size * 1.5 
        cx = wpose.position.x
        start_y = self.y_h
        
        # Posicionarse en la parte trasera del auto (inferior izquierda)
        wpose, waypoints = self.set_pen(wpose, waypoints, cx - 1.0*s, start_y, self.pen + 0.05)
        wpose, waypoints = self.down_pen(wpose, waypoints)
        
        # 1. Parachoques trasero (Sube recto)
        wpose, waypoints = self.set_pen(wpose, waypoints, cx - 1.0*s, start_y + 0.4*s, self.pen)
        
        # 2. Maletero (Línea horizontal)
        wpose, waypoints = self.set_pen(wpose, waypoints, cx - 0.7*s, start_y + 0.4*s, self.pen)
        
        # 3. Parabrisas trasero (Diagonal hacia arriba)
        wpose, waypoints = self.set_pen(wpose, waypoints, cx - 0.4*s, start_y + 0.9*s, self.pen)
        
        # 4. Techo (Línea horizontal)
        wpose, waypoints = self.set_pen(wpose, waypoints, cx + 0.2*s, start_y + 0.9*s, self.pen)
        
        # 5. Parabrisas delantero (Diagonal hacia abajo)
        wpose, waypoints = self.set_pen(wpose, waypoints, cx + 0.6*s, start_y + 0.4*s, self.pen)
        
        # 6. Capó / Hood (Línea horizontal)
        wpose, waypoints = self.set_pen(wpose, waypoints, cx + 1.0*s, start_y + 0.4*s, self.pen)
        
        # 7. Parachoques delantero (Baja recto)
        wpose, waypoints = self.set_pen(wpose, waypoints, cx + 1.0*s, start_y, self.pen)
        
        # 8. Chasis delantero hasta la primera llanta
        wpose, waypoints = self.set_pen(wpose, waypoints, cx + 0.7*s, start_y, self.pen)
        
        # 9. Llanta delantera (Arco trigonométrico de derecha a izquierda)
        wheel_radius = 0.2*s
        for theta_deg in range(0, 181, 15):
            theta_rad = math.radians(theta_deg)
            wx = (cx + 0.5*s) + wheel_radius * math.cos(theta_rad)
            wy = start_y + wheel_radius * math.sin(theta_rad)
            wpose, waypoints = self.set_pen(wpose, waypoints, wx, wy, self.pen)
            
        # 10. Chasis central (Línea horizontal entre las dos llantas)
        wpose, waypoints = self.set_pen(wpose, waypoints, cx - 0.3*s, start_y, self.pen)
        
        # 11. Llanta trasera (Arco trigonométrico)
        for theta_deg in range(0, 181, 15):
            theta_rad = math.radians(theta_deg)
            wx = (cx - 0.5*s) + wheel_radius * math.cos(theta_rad)
            wy = start_y + wheel_radius * math.sin(theta_rad)
            wpose, waypoints = self.set_pen(wpose, waypoints, wx, wy, self.pen)
            
        # 12. Cerrar silueta (Hasta el parachoques trasero original)
        wpose, waypoints = self.set_pen(wpose, waypoints, cx - 1.0*s, start_y, self.pen)
        
        # Levantar la herramienta al finalizar
        wpose, waypoints = self.up_pen(wpose, waypoints)
        
        return waypoints, wpose

    # =========================================================
    # 3. EL "MENÚ" INTELIGENTE Y DIBUJADO DE RVIZ
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

        if hasattr(self, figure_name):
            figure_function = getattr(self, figure_name)
            waypoints, wpose = figure_function(wpose, waypoints)
            
            # Publicar el trazo para verlo en RViz
            self.publish_trajectory_marker(waypoints)
            
            self.execute_drawing(waypoints)
        else:
            self.get_logger().warn(f"Error: La figura '{figure_name}' no existe en la clase.")

    def publish_trajectory_marker(self, waypoints):
        """Genera un Marker LINE_STRIP para visualizar la ruta en RViz."""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'trajectory_trace'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.005  # Grosor de la línea
        marker.color.r = 0.0
        marker.color.g = 1.0    # Color Verde
        marker.color.b = 0.0
        marker.color.a = 1.0    # Opacidad
        
        for wp in waypoints:
            p = Point()
            p.x = wp.position.x
            p.y = wp.position.y
            p.z = wp.position.z
            marker.points.append(p)
            
        self.marker_pub.publish(marker)
        self.get_logger().info("Marcador de trayectoria publicado para RViz.")

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

    start_pose = Pose()
    start_pose.position.x = 0.0
    start_pose.position.y = 0.2
    start_pose.position.z = node.pen + 0.00
    start_pose.orientation.x = 0.0
    start_pose.orientation.y = 1.0 
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 0.0

    # Llamamos a nuestra nueva figura de auto
    node.draw_figure("car_profile", start_pose)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
