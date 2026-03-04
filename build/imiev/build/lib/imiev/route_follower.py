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
from geometry_msgs.msg import PoseStamped


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

    def load_route(self):
        if not os.path.exists(self.route_file_path):
            self.get_logger().error(f"File {self.route_file_path} not found.")
            return False

        with open(self.route_file_path, 'r') as f:
            data = yaml.safe_load(f)

        if not data:
            self.get_logger().error("Empty route file.")
            return False

        if "waypoints" not in data or not data["waypoints"]:
            self.get_logger().error("No waypoints found in route file.")
            return False

        raw_points = list(data["waypoints"].values())

        # Filtrar puntos demasiado juntos (ruido por estar parados al grabar)
        self.gps_points = []
        min_distance = 0.5 # metros
        
        for pt in raw_points:
            if not self.gps_points:
                self.gps_points.append(pt)
            else:
                last_pt = self.gps_points[-1]
                dist = math.sqrt((pt["pose"][0] - last_pt["pose"][0])**2 + (pt["pose"][1] - last_pt["pose"][1])**2)
                if dist >= min_distance:
                    self.gps_points.append(pt)
                    
        # Asegurarnos de añadir el punto final
        if raw_points and raw_points[-1] != self.gps_points[-1]:
            last_raw = raw_points[-1]
            last_filtered = self.gps_points[-1]
            dist = math.sqrt((last_raw["pose"][0] - last_filtered["pose"][0])**2 + (last_raw["pose"][1] - last_filtered["pose"][1])**2)
            if dist > 0.1: # al menos 10 cm
                self.gps_points.append(last_raw)

        self.get_logger().info(f"Loaded {len(raw_points)} raw poses, filtered down to {len(self.gps_points)} valid waypoints.")
        
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
        
        # Add a LINE_STRIP to visualize the complete path natively
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "route_line"
        line_marker.id = 9999
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.15 # line width
        line_marker.color.r = 1.0 # Yellow
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.8
        
        for i, pt in enumerate(self.gps_points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "route_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            x = float(pt["pose"][0])
            y = float(pt["pose"][1])
            z = float(pt["pose"][2])
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            
            # Add to line strip
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            line_marker.points.append(p)
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Color coding: Green for start, Blue for end, Red for intermediate
            if i == 0: # Start
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1.0
            elif i == len(self.gps_points) - 1: # End
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; marker.color.a = 1.0
            else: # Waypoints
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 0.8
            
            marker_array.markers.append(marker)
            
        marker_array.markers.append(line_marker)
        self.marker_publisher.publish(marker_array)
        self.get_logger().info("Published route markers to Rviz.")

    def start_following(self):
        if not self.gps_points:
            return

        print("Waiting for Nav2 to be active...")
        self.navigator.waitUntilNav2Active(localizer='robot_localization')

        # Convert loaded poses to PoseStamped with dynamic orientations
        poses = []
        for i, pt in enumerate(self.gps_points):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            
            x = float(pt["pose"][0])
            y = float(pt["pose"][1])
            z = float(pt["pose"][2])
            
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            
            # Dinámicamente calcular la orientación para mirar al siguiente punto
            if i < len(self.gps_points) - 1:
                next_x = self.gps_points[i+1]["pose"][0]
                next_y = self.gps_points[i+1]["pose"][1]
                yaw = math.atan2(next_y - y, next_x - x)
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                # Para el último punto respetamos la orientación grabada en el yaml
                pose.pose.orientation.x = float(pt["orientation"][0])
                pose.pose.orientation.y = float(pt["orientation"][1])
                pose.pose.orientation.z = float(pt["orientation"][2])
                pose.pose.orientation.w = float(pt["orientation"][3])

            poses.append(pose)

        # Main loop to resume route if aborted (e.g. waypoint blocked)
        current_pose_index = 0
        total_poses = len(poses)
        stuck_timeout = 25.0
        stuck_threshold = 0.5

        while current_pose_index < total_poses:
            remaining_poses = poses[current_pose_index:]
            print(f"\n[ROUTE] Siguiendo ruta desde el waypoint {current_pose_index} ({len(remaining_poses)} waypoints restantes)...")
            
            self.navigator.goThroughPoses(remaining_poses)
            
            last_move_time = time.time()
            last_pos = self._get_robot_position()
            aborted_due_to_stuck = False

            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
                self.publish_markers()

                if not rclpy.ok():
                    self.navigator.cancelTask()
                    return

                now = time.time()
                pos = self._get_robot_position()

                feedback = self.navigator.getFeedback()
                if feedback and hasattr(feedback, 'number_of_poses_remaining'):
                    # Poses remaining in the *current* request
                    wp_current_local = len(remaining_poses) - feedback.number_of_poses_remaining
                    # Global waypoint index
                    wp_current = current_pose_index + wp_current_local
                    
                    print(f'\r>>> Dirigiéndose al Waypoint {wp_current}/{total_poses}  ', end='', flush=True)

                if pos[0] is not None and last_pos[0] is not None:
                    dist_moved = math.sqrt((pos[0] - last_pos[0])**2 + (pos[1] - last_pos[1])**2)
                    if dist_moved > stuck_threshold:
                        last_pos = pos
                        last_move_time = now
                    elif (now - last_move_time) > stuck_timeout:
                        print(f'\n🚫 Robot atascado completamente ({stuck_timeout:.0f}s sin moverse). Abortando tramo actual...')
                        self.navigator.cancelTask()
                        aborted_due_to_stuck = True
                        time.sleep(0.5)
                        break

                time.sleep(0.5)

            print() # Nueva línea
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                print('✅ Tarea de navegación completada con éxito!')
                break # All done!
                
            elif result == TaskResult.CANCELED and not aborted_due_to_stuck:
                print('⏭️  Ruta cancelada de forma manual')
                break
                
            else:
                print('⚠️  El planificador abortó (obstáculo irresoluble) o robot atascado.')
                
                # Determine where we are and skip to the NEXT feasible waypoint
                current_robot_pos = self._get_robot_position()
                if current_robot_pos[0] is None:
                    print("No se pudo obtener la posición del robot para reanudar.")
                    break
                    
                rx, ry = current_robot_pos
                
                # Find closest waypoint ahead of us
                closest_dist = float('inf')
                closest_index = current_pose_index
                
                # Search only in remaining waypoints
                for i in range(current_pose_index, total_poses):
                    wp_x = poses[i].pose.position.x
                    wp_y = poses[i].pose.position.y
                    dist = math.sqrt((rx - wp_x)**2 + (ry - wp_y)**2)
                    if dist < closest_dist:
                        closest_dist = dist
                        closest_index = i
                
                # We failed reaching/passing the closest one. Skip it and try the NEXT one.
                next_index = closest_index + 1
                
                if next_index >= total_poses:
                    print("❌ No quedan más waypoints accesibles. Fin de la ruta.")
                    break
                    
                print(f"🔄 Saltando waypoint infactible. Reanudando desde waypoint {next_index}...")
                current_pose_index = next_index
                time.sleep(1.0) # Pause before retrying

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