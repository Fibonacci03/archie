# ARCHIE: Manipulador Robótico de 6 GDL en ROS 2 Jazzy

![ROS 2](https://img.shields.io/badge/ros2-jazzy-blue?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?logo=gazebo)
![Dynamixel](https://img.shields.io/badge/Dynamixel-2XL430-red)
![Maintainer](https://img.shields.io/badge/Maintainer-David%20Torres-green)

**ARCHIE** es un ecosistema de simulación y control para un brazo robótico de 6 grados de libertad (GDL), desarrollado en **ROS 2 Jazzy** sobre Ubuntu 24.04. Soporta dos modos de operación: simulación física con **Gazebo Harmonic** y control de hardware real con motores **Dynamixel 2XL430** via interfaz U2D2. En ambos modos la capa de planificación (**MoveIt 2**) y la lógica de aplicación (`write_word`) funcionan sin modificaciones.

---

## Estructura del Proyecto

| Paquete | Descripción |
|---|---|
| `archie_description` | URDF/Xacro, mallas (meshes) y configuración de la descripción del robot |
| `archie_moveit2` | Configuración de MoveIt 2: OMPL, KDL, SRDF, joint limits, controladores |
| `archie_hardware` | Comunicación con hardware real: action server `FollowJointTrajectory` + publisher `/joint_states` via Dynamixel SDK |
| `archie_master` | Lógica de aplicación: escritura de palabras (`write_word_node`) y trazador de trayectoria (`archie_tracer`) |

La arquitectura de tópicos y acciones es idéntica en simulación y hardware real, por lo que `archie_master` y MoveIt 2 no distinguen entre ambos modos.

---

## Requisitos del Sistema

**Base (simulación y hardware real):**
```bash
sudo apt install ros-jazzy-moveit \
                 ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers \
                 ros-jazzy-control-msgs \
                 ros-jazzy-trajectory-msgs \
                 ros-jazzy-xacro \
                 ros-jazzy-tf2-ros \
                 ros-jazzy-visualization-msgs
```

**Solo simulación:**
```bash
sudo apt install ros-jazzy-ros-gz-bridge
```

**Solo hardware real:**
```bash
pip3 install dynamixel-sdk --user
# Permisos de puerto USB (requiere cerrar sesión y volver a entrar)
sudo usermod -aG dialout $USER
```

---

## Instalación y Compilación

```bash
mkdir -p ~/ramel_ws/src
cd ~/ramel_ws/src
git clone <URL_DEL_REPOSITORIO> .
cd ~/ramel_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Ejecución — Simulación (Gazebo Harmonic)

Ejecutar `source ~/ramel_ws/install/setup.bash` en cada terminal antes de lanzar.

**Terminal 1 — Gazebo (entorno físico virtual):**
```bash
ros2 launch archie_description spawn.launch.py
```

**Terminal 2 — MoveIt 2 + trazador:**
```bash
ros2 launch archie_master archie_bringup.launch.py use_sim:=true
```

**Terminal 3 — RViz (visualización y planificación):**
```bash
ros2 launch archie_moveit2 moveit_rviz.launch.py
```

**Terminal 4 — Escritura de palabra** (ejecutar solo cuando Gazebo y MoveIt estén listos):
```bash
ros2 run archie_master write_word_node
```

Al terminar la escritura el robot intenta volver a la posición inicial. En simulación el servicio `/go_home` no está disponible, por lo que se registra una advertencia y el robot queda en la última posición.

---

## Ejecución — Hardware Real (Dynamixel 2XL430 via U2D2)

Ejecutar `source ~/ramel_ws/install/setup.bash` en cada terminal antes de lanzar.

**Terminal 1 — Hardware Dynamixel:**
```bash
ros2 launch archie_hardware real_bringup.launch.py
```

Al arrancar, el nodo lee la posición actual de los motores y **no mueve el robot**. Argumentos opcionales:

```bash
# Puerto USB distinto al default /dev/ttyUSB0
ros2 launch archie_hardware real_bringup.launch.py usb_port:=/dev/ttyUSB1

# Mover suavemente a home (0 rad en todos los joints) al arrancar
ros2 launch archie_hardware real_bringup.launch.py go_home_on_start:=true

# Ajustar perfil de movimiento (más lento = más seguro)
ros2 launch archie_hardware real_bringup.launch.py profile_velocity:=20 profile_acceleration:=5
```

**Terminal 2 — MoveIt 2 + trazador:**
```bash
ros2 launch archie_master archie_bringup.launch.py use_sim:=false
```

**Terminal 3 — RViz (visualización en tiempo real del robot real):**
```bash
ros2 launch archie_moveit2 moveit_rviz.launch.py
```

RViz muestra el modelo 3D del robot reflejando las posiciones reales leídas de los motores Dynamixel, y la trayectoria planeada por MoveIt 2 antes de ejecutarla.

**Terminal 4 — Escritura de palabra** (ejecutar solo cuando los motores y MoveIt estén listos):
```bash
ros2 run archie_master write_word_node
```

Al terminar la escritura, el robot vuelve automáticamente a la posición inicial (todos los joints a 0 rad) de forma suave.

**Volver a home manualmente** en cualquier momento:
```bash
ros2 service call /go_home std_srvs/srv/Trigger {}
```

---

## Perfil de Movimiento Dynamixel

Los motores usan un **perfil trapezoidal** de velocidad/aceleración a nivel de firmware, más **interpolación lineal a 50 Hz** en el nodo de hardware. Esto produce movimientos suaves sin sacudidas.

| Parámetro | Unidad | Default | Descripción |
|---|---|---|---|
| `profile_velocity` | 0.229 rpm | 30 (~6.9 rpm) | Velocidad máxima de cada joint |
| `profile_acceleration` | ~214.6 rpm² | 8 | Suavidad de la rampa de arranque y freno |

Usar valor `0` elimina el límite → movimiento brusco, no recomendado.

---

## Solución de Problemas

### Motor Dynamixel titila en rojo
Indica un error de hardware. El motor desactiva su torque automáticamente como protección.

Causas más frecuentes:

| Error | Causa | Solución |
|---|---|---|
| **Overload** (más común) | Brazo bloqueado físicamente, peso excesivo, o `profile_velocity` muy alto | Mover a posición neutral manualmente, reducir `profile_velocity` |
| **Overheating** | Uso intensivo prolongado | Dejar enfriar y rebootear |
| **Input Voltage** | Voltaje fuera de rango (el 2XL430 opera entre 11–14.8 V) | Revisar fuente de alimentación |

Diagnóstico por código de error (registro 70):
```python
hw_error, _, _ = packetHandler.read1ByteTxRx(portHandler, motor_id, 70)
# Bit 0: Input Voltage | Bit 2: Overheating | Bit 3: Encoder
# Bit 4: Electrical Shock | Bit 5: Overload
```

Para limpiar el error tras corregir la causa:
```python
packetHandler.reboot(portHandler, motor_id)
```

> Descansar el motor solo ayuda si el error es térmico. Para Overload, el error reaparece inmediatamente si no se corrige la posición o la carga.

### Plan al 0% — "Found empty JointState message"
MoveIt no tiene el estado articular al planear. Verificar que haya datos en `/joint_states`:
```bash
ros2 topic echo /joint_states --once
```
El nodo `write_word_node` espera automáticamente a recibir `/joint_states` antes de planear. Si el problema persiste, verificar que `archie_hardware_node` (real) o Gazebo (sim) estén activos.

### Conflicto de Locale (Segmentation Fault)
En sistemas con configuración regional en español, ROS 2 puede fallar al interpretar decimales.
```bash
export LC_NUMERIC="C"
```

### RViz no carga en Ubuntu 24.04 (Wayland)
```bash
export QT_QPA_PLATFORM=xcb
```

### Tipado estricto en YAML
ROS 2 Jazzy no convierte `int` a `double` implícitamente. Los valores de velocidad y aceleración en `joint_limits.yaml` deben declararse con punto decimal: `3.0` en lugar de `3`.

---

## Autor
**David Torres** – Ingeniero en Mecatrónica (ESPOL).  
Técnico de Laboratorio - CoRAL - _Collaborative Robotics and Artificial Intelligence Laboratory_  
Investigador Robótica de Enjambre, Navegación Autónoma y Automatización Industrial.
