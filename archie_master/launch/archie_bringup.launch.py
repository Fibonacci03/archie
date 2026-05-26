from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Declarar el argumento global (Por defecto: simulador activado)
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Usar tiempo de simulación (true para Gazebo, false para hardware real)'
    )

    # 2. Capturar el valor para pasarlo a los nodos
    use_sim_time = LaunchConfiguration('use_sim')

    # Nodo trazador — visualiza la trayectoria en RViz
    tracer_node = Node(
        package='archie_master',
        executable='archie_tracer',
        name='archie_tracer_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # MoveIt
    moveit_launch_dir = os.path.join(get_package_share_directory('archie_moveit2'), 'launch')
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_launch_dir, 'move_group.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # NOTA: write_word_node NO se lanza aquí.
    # Para escribir, ejecutar por separado:
    #   ros2 run archie_master write_word_node
    return LaunchDescription([
        use_sim_arg,
        moveit_launch,
        tracer_node
    ])