import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import yaml
import os
import sys
import time
import math
from imiev.utils.gps_utils import latLonYaw2Geopose


class RouteFollower(Node):
    """
    Class to follow a route and visualize points in RViz
    """

    def __init__(self, route_file_path):
        super().__init__('route_follower')
        self.navigator = BasicNavigator("basic_navigator")
        self.route_file_path = route_file_path
        self.gps_points = []
        
        # QoS for latching (transient local)
        qos_profile = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.marker_publisher = self.create_publisher(MarkerArray, '/route_markers', qos_profile)
        
        # Origin for Sonoma Raceway to convert lat/lon to local X/Y for visualization
        self.origin_lat = 38.161479
        self.origin_lon = -122.454630
        self.earth_radius = 6371000.0

    def lat_lon_to_local(self, lat, lon):
        """
        Simple lat/lon to local X/Y conversion for visualization
        """
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)

        delta_lat = lat_rad - origin_lat_rad
        delta_lon = lon_rad - origin_lon_rad

        x = delta_lon * self.earth_radius * math.cos(origin_lat_rad)
        y = delta_lat * self.earth_radius
        return x, y

    def load_route(self):
        if not os.path.exists(self.route_file_path):
            self.get_logger().error(f"File {self.route_file_path} not found.")
            return False

        with open(self.route_file_path, 'r') as f:
            data = yaml.safe_load(f)

        if not data:
            self.get_logger().error("Empty route file.")
            return False

        temp_gps = []
        # Compile start, waypoints, and end
        if "start" in data and data["start"]:
            temp_gps.append(data["start"])
        if "waypoints" in data and data["waypoints"]:
            temp_gps.extend(data["waypoints"])
        if "end" in data and data["end"]:
            temp_gps.append(data["end"])

        self.gps_points = temp_gps
        self.get_logger().info(f"Loaded {len(self.gps_points)} points from route.")
        
        # Initial publications to ensure symbols appear in Rviz
        for _ in range(5):
            self.publish_markers()
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
            
        return True

    def publish_markers(self):
        """
        Publishes markers to RViz
        """
        marker_array = MarkerArray()
        
        for i, pt in enumerate(self.gps_points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "route"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            x, y = self.lat_lon_to_local(pt["latitude"], pt["longitude"])
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.5
            
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            
            # Color coding: Green for start, Blue for end, Red for intermediate
            if i == 0: # Start
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1.0
            elif i == len(self.gps_points) - 1: # End
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; marker.color.a = 1.0
            else: # Waypoints
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 0.8
            
            marker_array.markers.append(marker)
            
        self.marker_publisher.publish(marker_array)
        self.get_logger().info("Published route markers to Rviz.")

    def start_following(self):
        if not self.gps_points:
            return

        geoposes = [latLonYaw2Geopose(p["latitude"], p["longitude"], p["yaw"]) for p in self.gps_points]

        print("Waiting for Nav2 to be active...")
        self.navigator.waitUntilNav2Active(localizer='robot_localization')

        # Coordenadas locales de cada waypoint para comparar distancias
        local_targets = [self.lat_lon_to_local(p["latitude"], p["longitude"]) for p in self.gps_points]

        current_wp = 0
        total = len(geoposes)

        # --- Parámetros de atasco ---
        stuck_timeout = 25.0       # Sin moverse durante X segundos → abortar
        stuck_threshold = 0.5      # Movimiento mínimo para no estar "atascado" (m)

        print(f"Siguiendo ruta completa ({total} waypoints) vía Through Poses...")
        
        # Enviar TODA la ruta de golpe para que Nav2 fluya sin parar
        self.navigator.followGpsWaypoints(geoposes)

        last_move_time = time.time()
        last_pos = self._get_robot_position()

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.publish_markers()

            if not rclpy.ok():
                self.navigator.cancelTask()
                return

            now = time.time()
            pos = self._get_robot_position()

            # Obtener y mostrar feedback de progreso
            feedback = self.navigator.getFeedback()
            if feedback and hasattr(feedback, 'number_of_poses_remaining'):
                wp_current = total - feedback.number_of_poses_remaining
                # Use carriage return `\r` to overwrite the same line en la consola
                print(f'\r>>> Dirigiéndose al Waypoint {wp_current}/{total}  ', end='', flush=True)

            # Detección de robot atascado
            if pos[0] is not None and last_pos[0] is not None:
                dist_moved = math.sqrt((pos[0] - last_pos[0])**2 + (pos[1] - last_pos[1])**2)
                if dist_moved > stuck_threshold:
                    last_pos = pos
                    last_move_time = now
                elif (now - last_move_time) > stuck_timeout:
                    print(f'\n🚫 Robot atascado completamente '
                          f'({stuck_timeout:.0f}s sin moverse), abortando ruta...')
                    self.navigator.cancelTask()
                    time.sleep(0.5)
                    break

            time.sleep(0.5)

        print() # Nueva línea al terminar el bucle del carriage return
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('✅ Ruta completada con éxito!')
        elif result == TaskResult.CANCELED:
            print('⏭️  Ruta cancelada de forma manual o por atasco')
        else:
            print('⚠️  La ruta finalizó con errores.')

    def _get_robot_position(self):
        """Posición del robot (x, y) en frame map via TF."""
        try:
            from tf2_ros import Buffer, TransformListener
            if not hasattr(self, '_tf_buffer'):
                self._tf_buffer = Buffer()
                self._tf_listener = TransformListener(self._tf_buffer, self)
            t = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None, None


def main():
    rclpy.init()

    yaml_input = sys.argv[1] if len(sys.argv) > 1 else "route"

    # Decide if the input is a full path or a name in the 'rutas' directory
    if yaml_input.endswith(".yaml") or "/" in yaml_input:
        yaml_path = yaml_input
    else:
        # Search in the standard 'rutas' folder
        try:
            package_path = os.path.join(os.path.expanduser("~"), "simulador_ws/imiev/src/imiev")
            yaml_path = os.path.join(package_path, "rutas", f"{yaml_input}.yaml")
        except Exception:
            yaml_path = os.path.expanduser(f"~/rutas/{yaml_input}.yaml")

    follower = RouteFollower(yaml_path)
    
    if follower.load_route():
        try:
            follower.start_following()
        except KeyboardInterrupt:
            follower.navigator.cancelTask()
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
