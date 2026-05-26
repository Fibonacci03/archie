from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Nodo del Trazador (El que dibuja la línea)
    tracer_node = Node(
        package='archie_master',
        executable='archie_tracer',
        name='archie_tracer_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 2. Incluir el Launch de MoveIt RViz (El que abre la ventana)
    # Suponiendo que usas el generado por MoveIt Setup Assistant
    rviz_launch_dir = os.path.join(get_package_share_directory('archie_moveit2'), 'launch')
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rviz_launch_dir, 'moveit_rviz.launch.py'))
    )

    return LaunchDescription([
        tracer_node,
        rviz_launch
    ])