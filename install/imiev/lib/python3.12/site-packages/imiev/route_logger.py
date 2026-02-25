import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
import yaml
import os
import sys
import tkinter as tk
from tkinter import messagebox
from imiev.utils.gps_utils import euler_from_quaternion


class RouteLogger(tk.Tk, Node):
    """
    ROS2 node to log a route (start, intermediate, end) to a file
    """

    def __init__(self, logging_file_path):
        tk.Tk.__init__(self)
        Node.__init__(self, 'route_logger')
        self.title("Route Logger")

        self.logging_file_path = logging_file_path
        self.recording = False
        self.route_data = {
            "start": None,
            "waypoints": [],
            "end": None
        }

        # GUI Setup
        self.gps_pose_label = tk.Label(self, text="Current Coordinates:")
        self.gps_pose_label.pack(pady=5)
        self.gps_pose_textbox = tk.Label(self, text="Waiting for GPS...", width=50)
        self.gps_pose_textbox.pack(pady=5)

        self.status_label = tk.Label(self, text="Status: Idle", fg="blue")
        self.status_label.pack(pady=5)

        # Route Name Setup
        tk.Label(self, text="Route Name:").pack(pady=2)
        self.name_entry = tk.Entry(self)
        self.name_entry.insert(0, "route1")
        self.name_entry.pack(pady=2)

        self.start_button = tk.Button(self, text="Start Recording (Start Point)",
                                     command=self.start_recording, bg="green", fg="white")
        self.start_button.pack(fill=tk.X, padx=10, pady=2)

        self.log_button = tk.Button(self, text="Log Manual Waypoint",
                                   command=self.log_manual_waypoint, state=tk.DISABLED)
        self.log_button.pack(fill=tk.X, padx=10, pady=2)

        self.stop_button = tk.Button(self, text="Stop Recording (End Point)",
                                    command=self.stop_recording, state=tk.DISABLED, bg="red", fg="white")
        self.stop_button.pack(fill=tk.X, padx=10, pady=2)

        # ROS Setup
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            1
        )
        self.last_gps_position = NavSatFix()

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            1
        )
        self.last_heading = 0.0

        # Periodic logging timer (e.g., every 5 seconds if recording)
        self.timer = self.create_timer(5.0, self.periodic_log_callback)

    def gps_callback(self, msg: NavSatFix):
        self.last_gps_position = msg
        self.update_gui_text()

    def imu_callback(self, msg: Imu):
        _, _, self.last_heading = euler_from_quaternion(msg.orientation)
        self.update_gui_text()

    def update_gui_text(self):
        self.gps_pose_textbox.config(
            text=f"Lat: {self.last_gps_position.latitude:.6f}, Lon: {self.last_gps_position.longitude:.6f}, Heading: {self.last_heading:.2f} rad")

    def start_recording(self):
        if self.last_gps_position.latitude == 0.0:
            messagebox.showwarning("Warning", "GPS data not yet available.")
            return

        self.recording = True
        self.route_data["start"] = {
            "latitude": self.last_gps_position.latitude,
            "longitude": self.last_gps_position.longitude,
            "yaw": self.last_heading
        }
        self.route_data["waypoints"] = []
        self.route_data["end"] = None

        self.status_label.config(text="Status: Recording...", fg="red")
        self.start_button.config(state=tk.DISABLED)
        self.log_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.NORMAL)
        self.get_logger().info("Started recording route.")

    def log_manual_waypoint(self):
        if not self.recording:
            return
        self.add_waypoint("manual")

    def periodic_log_callback(self):
        if self.recording:
            self.add_waypoint("periodic")

    def add_waypoint(self, type_str):
        wp = {
            "latitude": self.last_gps_position.latitude,
            "longitude": self.last_gps_position.longitude,
            "yaw": self.last_heading,
            "type": type_str
        }
        self.route_data["waypoints"].append(wp)
        self.get_logger().info(f"Logged {type_str} waypoint.")

    def stop_recording(self):
        if not self.recording:
            return

        self.recording = False
        self.route_data["end"] = {
            "latitude": self.last_gps_position.latitude,
            "longitude": self.last_gps_position.longitude,
            "yaw": self.last_heading
        }

        self.save_to_file()

        self.status_label.config(text="Status: Idle (Saved)", fg="blue")
        self.start_button.config(state=tk.NORMAL)
        self.log_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.DISABLED)
        self.get_logger().info("Stopped recording and saved route.")

    def save_to_file(self):
        try:
            # If the default path was meant to be used, use the name from the entry
            if "route.yaml" in self.logging_file_path:
                route_name = self.name_entry.get().strip()
                if not route_name:
                    route_name = "route"
                
                directory = os.path.dirname(self.logging_file_path)
                final_path = os.path.join(directory, f"{route_name}.yaml")
            else:
                final_path = self.logging_file_path

            os.makedirs(os.path.dirname(final_path), exist_ok=True)
            with open(final_path, 'w') as yaml_file:
                yaml.dump(self.route_data, yaml_file, default_flow_style=False)
            messagebox.showinfo("Success", f"Route saved to:\n{final_path}")
        except Exception as ex:
            messagebox.showerror("Error", f"Failed to save route: {str(ex)}")


def main(args=None):
    rclpy.init(args=args)

    # Intentar encontrar la ruta del paquete imiev para guardar allí por defecto
    try:
        from ament_index_python.packages import get_package_share_directory
        # Usamos la carpeta del código fuente si es posible, o una en el home para evitar permisos
        package_path = os.path.join(os.path.expanduser("~"), "simulador_ws/imiev/src/imiev")
        if not os.path.exists(package_path):
             package_path = os.path.expanduser("~")
        
        default_path = os.path.join(package_path, "rutas", "route.yaml")
    except Exception:
        default_path = os.path.expanduser("~/rutas/route.yaml")

    yaml_path = sys.argv[1] if len(sys.argv) > 1 else default_path

    route_logger = RouteLogger(yaml_path)

    try:
        while rclpy.ok():
            rclpy.spin_once(route_logger, timeout_sec=0.1)
            route_logger.update()
    except KeyboardInterrupt:
        pass
    finally:
        route_logger.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
