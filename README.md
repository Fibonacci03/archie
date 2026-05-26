# ARCHIE: Manipulador Robótico de 6 GDL en ROS 2 Jazzy

![ROS 2](https://img.shields.io/badge/ros2-jazzy-blue?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?logo=gazebo)
![Maintainer](https://img.shields.io/badge/Maintainer-David%20Torres-green)

**ARCHIE** es un ecosistema de simulación y control para un brazo robótico de 6 grados de libertad (GDL), migrado y optimizado para **Ubuntu 24.04**. El proyecto integra planificación de trayectorias con **MoveIt 2**, simulación física en **Gazebo Harmonic** y una herramienta personalizada de trazado de rutas para análisis cinemático.

---

## 🛠 Requisitos del Sistema

Antes de clonar, asegúrate de tener instalado el siguiente stack tecnológico:

* **OS:** Ubuntu 24.04 LTS (Noble Numbat).
* **ROS 2:** Jazzy Jalisco (Desktop Install).
* **Simulador:** Gazebo Harmonic (GZ Sim).
* **Librerías clave:**
    ```bash
    sudo apt install ros-jazzy-moveit \
                     ros-jazzy-ros-gz-bridge \
                     ros-jazzy-ros2-control \
                     ros-jazzy-ros2-controllers \
                     ros-jazzy-tf2-ros \
                     ros-jazzy-visualization-msgs
    ```

---

## Instalación y Compilación

1. **Crear y clonar el workspace:**
   ```bash
   mkdir -p ~/ramel_ws/archie_ws/src
   cd ~/ramel_ws/archie_ws/src
   # Clonar el repositorio dentro de src
   git clone <URL_DEL_REPOSITORIO> .
   ```

2. **Instalar dependencias del paquete:**
   Desde la raíz del workspace, deja que `rosdep` resuelva las dependencias faltantes:
   ```bash
   cd ~/ramel_ws/archie_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Compilación optimizada:**
   Se recomienda usar `--symlink-install` para permitir cambios en archivos Python y configuraciones YAML sin necesidad de recompilar.
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## Estructura del Proyecto

* **`archie_description`**: Contiene los archivos URDF/Xacro, mallas (meshes) y configuraciones de `ros2_control` para la integración con Gazebo. Incluye el nodo `archie_tracer.py`.
* **`archie_moveit2`**: Paquete de configuración de MoveIt (OMPL, KDL, MoveGroup) generado para la cinemática del brazo.
* **`archie_moveit_config`**: Archivos de configuración adicionales para la integración entre MoveIt y Gazebo.

---

## 🎮 Ejecución

Para un funcionamiento correcto, abre tres terminales y ejecuta `source install/setup.bash` en cada una.

### 1. Entorno Físico (Gazebo Harmonic)
Lanza el mundo y carga al robot con sus controladores de hardware virtual:
```bash
ros2 launch archie_description spawn.launch.py
```

### 2. Cerebro Cinemático (MoveIt 2)
Carga el servidor de planificación sincronizado con el reloj de la simulación:
```bash
ros2 launch archie_moveit2 move_group.launch.py
```

### 3. Interfaz de Control (RViz)
Visualiza el robot y manipula los *Interactive Markers* para planificar rutas:
```bash
ros2 launch archie_moveit2 moveit_rviz.launch.py
```

### 4. Trazado de Trayectorias (Opcional)
Para visualizar la estela del efector final en tiempo real:
```bash
ros2 run archie_description archie_tracer
```

---

## Solución de Problemas (Troubleshooting)

Durante el despliegue en ROS 2 Jazzy, se identificaron y resolvieron los siguientes puntos críticos:

### 1. Conflicto de Formato Numérico (Locale)
En sistemas con configuración regional en español, ROS 2 puede fallar al interpretar decimales (usando comas en lugar de puntos), provocando *Segmentation Faults*.
**Solución:** Forzar el estándar C en la terminal:
```bash
export LC_NUMERIC="C"
```

### 2. Sincronización de Reloj (Simulation Time)
Si MoveIt aborta la ejecución con errores de *Timeout*, se debe a un desfase entre el tiempo real y el de simulación. 
**Solución:** Los archivos Launch de este repositorio están modificados para inyectar automáticamente `use_sim_time: True` en los nodos de `move_group` y `rviz2`.

### 3. Tipado Estricto en YAML
ROS 2 Jazzy no permite la conversión implícita de `int` a `double`.
**Solución:** Todos los valores en `joint_limits.yaml` (velocity, acceleration) deben declararse con punto decimal (ej. `3.0` en lugar de `3`).

### 4. Renderizado Gráfico (Wayland vs X11)
Si la interfaz de RViz no carga correctamente en Ubuntu 24.04, fuerza el uso del sistema XCB:
```bash
export QT_QPA_PLATFORM=xcb
```

---

## Autor
**David Torres** – Ingeniero en Mecatrónica (ESPOL).  
Especialista en Robótica de Enjambre, Navegación Autónoma y Automatización Industrial.
