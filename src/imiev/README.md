# 🚗 iMiEV — Simulador ROS2 + Gazebo

Simulación del Mitsubishi i-MiEV con Nav2, SLAM, y navegación GPS.

## Compilar

```bash
cd ~/simulador_ws/imiev
colcon build --packages-select imiev --symlink-install
source install/setup.bash
```

---

## 🚀 Launch Files

### 1. Simulación base (Gazebo + Robot)
```bash
ros2 launch imiev gazebo_model.launch.py
```
Lanza Gazebo con el mundo Sonoma Raceway, spawn del robot, `robot_state_publisher`, y bridge ROS↔Gazebo.

---

### 2. Navegación GPS (sin mapa)
```bash
ros2 launch imiev gps_waypoint_follower.launch.py use_rviz:=True
```
Lanza todo lo necesario para navegación GPS: Gazebo + EKF (localización dual) + Nav2 (usa `nav2_no_map_params.yaml`).
Opciones: `use_rviz:=True`, `use_mapviz:=True`.

---

### 3. Navegación SLAM (con mapa)
```bash
ros2 launch imiev slam.launch.py
```
Lanza Gazebo + EKF local + SLAM Toolbox + Nav2 + RViz. Usa `nav2_slam_params.yaml`. Construye el mapa mientras navega.

---

### 4. Localización dual EKF
```bash
ros2 launch imiev dual_ekf_navsat.launch.py
```
Lanza los nodos de `robot_localization`: EKF local (odom → base_footprint), EKF global, y `navsat_transform` para GPS.

---

### 5. MapViz
```bash
ros2 launch imiev mapviz.launch.py
```
Lanza MapViz para visualización sobre mapa satélite.

---

## 🐍 Scripts (ros2 run)

### Grabar ruta GPS
```bash
ros2 run imiev route_logger
# O con ruta personalizada:
ros2 run imiev route_logger /ruta/al/archivo.yaml
```
GUI con tkinter para grabar rutas GPS. Registra punto de inicio, waypoints intermedios (automáticos cada 5s + manuales), y punto final. Guarda en `~/simulador_ws/imiev/src/imiev/rutas/`.

---

### Seguir ruta GPS
```bash
ros2 run imiev route_follower nombre_ruta
# O con ruta completa:
ros2 run imiev route_follower /ruta/al/archivo.yaml
```
Sigue una ruta guardada con `route_logger`. Muestra markers de la ruta en RViz (verde=inicio, rojo=intermedio, azul=fin).

---

### Grabar waypoints GPS individuales
```bash
ros2 run imiev gps_waypoint_logger
# O con ruta personalizada:
ros2 run imiev gps_waypoint_logger /ruta/al/archivo.yaml
```
GUI para grabar waypoints GPS individuales. Guarda en formato compatible con `logged_waypoint_follower`. Por defecto guarda en `~/mis_waypoints/gps_waypoints.yaml`.

---

### Seguir waypoints GPS
```bash
ros2 run imiev logged_waypoint_follower
# O con archivo personalizado:
ros2 run imiev logged_waypoint_follower /ruta/al/archivo.yaml
```
Sigue waypoints GPS guardados con `gps_waypoint_logger`. Por defecto lee de `~/mis_waypoints/gps_waypoints.yaml`.

---

### Waypoints interactivos (MapViz)
```bash
ros2 run imiev interactive_waypoint_follower
```
Recibe puntos GPS clicados en MapViz (`/clicked_point` en frame `wgs84`) y envía el robot directamente a esa posición.

---

### Diagnóstico de giro
```bash
ros2 run imiev test_turning
```
Publica `cmd_vel` con giro derecha e izquierda durante 5s cada uno. Mide yaw rate y ángulos de steering para verificar simetría del vehículo.

---

## 🛠️ Comandos útiles en runtime

### Navegar con clic en RViz (más fácil)
En la barra de RViz, pulsa **"2D Goal Pose"** (botón con flecha verde) y haz clic+arrastra en el mapa. La dirección del arrastre define la orientación final. El robot navega automáticamente.

### Navegar a un punto (terminal)
```bash
# 1. Obtener posición actual
ros2 run tf2_ros tf2_echo map base_footprint --once

# 2. Enviar goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.2, y: -3.1, z: 0.0}, orientation: {z: 0.15, w: 0.99}}}}"
```

### Mover el robot directamente (sin Nav2)
```bash
# Avanzar recto
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" -r 10

# Girar a la derecha
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: -0.2}}" -r 10

# Girar a la izquierda
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.2}}" -r 10
```

### Verificar parámetros del costmap
```bash
# Ver inflation_radius actual
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius

# Cambiar en caliente (sin reiniciar)
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 3.5
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 3.5
```

### Ver topics activos
```bash
ros2 topic list
ros2 topic echo /odom           # Odometría
ros2 topic echo /scan           # LiDAR
ros2 topic echo /gps/fix        # GPS
ros2 topic echo /imu/data       # IMU
```

---

## 📁 Archivos de configuración

| Archivo | Descripción |
|:---|:---|
| `config/nav2_no_map_params.yaml` | Parámetros Nav2 para navegación GPS (sin mapa) — usa **MPPI controller** |
| `config/nav2_slam_params.yaml` | Parámetros Nav2 para SLAM — usa **Regulated Pure Pursuit** |
| `config/dual_ekf_navsat_params.yaml` | Parámetros del EKF (robot_localization) |
| `config/mapper_params_online_async.yaml` | Parámetros de SLAM Toolbox |
| `config/imiev_bridge.yaml` | Configuración del bridge ROS↔Gazebo (topics) |
| `models/robot.xacro` | URDF del robot (dimensiones, ruedas, sensores) |
| `models/robot.gazebo` | Plugins de Gazebo (Ackermann steering, sensores) |

---

## ⚙️ Parámetros clave del vehículo

| Parámetro | Valor | Fuente |
|:---|:---|:---|
| Largo × Ancho × Alto | 3.56 × 1.86 × 1.64 m | `robot.xacro` |
| Masa | 1080 kg | `robot.xacro` |
| Radio de rueda | 0.30 m | `robot.xacro` |
| Distancia entre ejes | 2.50 m | `robot.gazebo` |
| Límite de dirección | 0.51 rad (29.2°) | `robot.xacro` |
| Radio mínimo de giro | 4.5 m | `robot.gazebo` / Nav2 |
| Velocidad máxima | 2.5 m/s | `robot.gazebo` |
