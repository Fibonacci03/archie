from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 1. Cargamos la configuración de ARCHIE
    moveit_config = MoveItConfigsBuilder("archie", package_name="archie_moveit2").to_moveit_configs()

    # 2. Extraemos los parámetros y forzamos el tiempo de simulación
    rviz_params = moveit_config.to_dict()
    rviz_params['use_sim_time'] = True

    # 3. Definimos el nodo de RViz manualmente
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        # Aquí le pasamos el archivo de configuración .rviz que generó el Setup Assistant
        arguments=["-d", os.path.join(moveit_config.package_path, "config", "moveit.rviz")],
        parameters=[rviz_params],
    )

    return LaunchDescription([rviz_node])