import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
import tf2_ros

class ArchieTracer(Node):
    def __init__(self):
        super().__init__('archie_tracer')
        self.marker_pub = self.create_publisher(Marker, '/archie_path_visual', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.srv = self.create_service(Empty, 'clear_trace', self.clear_callback)
                   
        # Configuración del Marcador
        self.marker = Marker()
        self.marker.header.frame_id = "world"
        self.marker.type = Marker.POINTS # Usamos puntos para evitar rayas en los saltos
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.004 # Tamaño del punto (4mm)
        self.marker.scale.y = 0.004
        self.marker.color.r = 1.0
        self.marker.color.a = 1.0 # Opaco
        
        self.timer = self.create_timer(0.02, self.update_trace) # 50Hz para mayor resolución

    def update_trace(self):
        try:
            # 1. Obtenemos la posición cinemática de link_6 (la brida)
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('world', 'link_6', now)
            
            # --- CORRECCIÓN 1: OFFSET CINEMÁTICO ---
            pencil_length = 0.135 # Ajustar según lápiz
            pos_x = trans.transform.translation.x
            pos_y = trans.transform.translation.y
            pos_z = trans.transform.translation.z - pencil_length # Proyección hacia abajo
            pen_down_z = 0.01 # Ajusta según tu cinemática real
            tolerance = 0.003 # 5mm de tolerancia
            
            if pos_z <= (pen_down_z + tolerance):
                # SOLO AQUÍ guardamos el punto para que el rastro sea 2D
                p = Point()
                p.x = pos_x
                p.y = pos_y
                p.z = 0.0 # Pegamos el rastro al piso (ej. a 1cm de altura)
                
                self.marker.points.append(p)
                
                # Publicamos la actualización
                self.marker.header.stamp = self.get_clock().now().to_msg()
                self.marker_pub.publish(self.marker)
                
        except Exception:
            pass

    def clear_callback(self, request, response):
        self.get_logger().info("Orden de limpieza recibida. Borrando marcadores...")
        
        # 1. Vaciamos la memoria interna del trazador
        self.marker.points.clear() 
        
        # 2. Le decimos a RViz que borre todo lo que tiene en pantalla
        clear_msg = Marker()
        clear_msg.header.frame_id = "world"
        clear_msg.action = Marker.DELETEALL
        self.marker_pub.publish(clear_msg)
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArchieTracer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()