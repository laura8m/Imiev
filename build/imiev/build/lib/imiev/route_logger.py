import rclpy
from rclpy.node import Node
import yaml
import os
import sys
import tkinter as tk
from tkinter import messagebox
from tf2_ros import Buffer, TransformListener

class RouteLogger(tk.Tk, Node):
    """
    ROS2 node to log a route (start, intermediate, end) to a file based on TF poses
    """

    def __init__(self, logging_file_path):
        tk.Tk.__init__(self)
        Node.__init__(self, 'route_logger')
        self.title("Route Logger")

        self.logging_file_path = logging_file_path
        self.recording = False
        
        self.route_data = {
            "waypoints": {}
        }
        self.waypoint_counter = 0

        # Output frames
        self.target_frame = 'map'
        self.source_frame = 'base_footprint'

        # GUI Setup
        self.pose_label = tk.Label(self, text="Current Coordinates (Map):")
        self.pose_label.pack(pady=5)
        self.pose_textbox = tk.Label(self, text="Waiting for TF...", width=50)
        self.pose_textbox.pack(pady=5)

        self.status_label = tk.Label(self, text="Status: Idle", fg="blue")
        self.status_label.pack(pady=5)

        # Route Name Setup
        tk.Label(self, text="Route Name:").pack(pady=2)
        self.name_entry = tk.Entry(self)
        self.name_entry.insert(0, "route_obs")
        self.name_entry.pack(pady=2)

        self.start_button = tk.Button(self, text="Start Recording",
                                     command=self.start_recording, bg="green", fg="white")
        self.start_button.pack(fill=tk.X, padx=10, pady=2)

        self.log_button = tk.Button(self, text="Log Manual Waypoint",
                                    command=self.log_manual_waypoint, state=tk.DISABLED)
        self.log_button.pack(fill=tk.X, padx=10, pady=2)

        self.stop_button = tk.Button(self, text="Stop Recording & Save",
                                     command=self.stop_recording, state=tk.DISABLED, bg="red", fg="white")
        self.stop_button.pack(fill=tk.X, padx=10, pady=2)

        # TF Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Cache last transform
        self.last_pose = None

        # Create timer to update GUI and fetch TF at 10Hz
        self.tf_timer = self.create_timer(0.1, self.fetch_transform)
        
        # Periodic logging timer (every 2 seconds if recording)
        self.timer = self.create_timer(2.0, self.periodic_log_callback)

    def fetch_transform(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time())
            
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            
            self.last_pose = {
                "pose": [float(x), float(y), float(z)],
                "orientation": [float(qx), float(qy), float(qz), float(qw)]
            }
            
            self.pose_textbox.config(
                text=f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}\nQ: ({qx:.2f}, {qy:.2f}, {qz:.2f}, {qw:.2f})")
        except Exception as e:
            self.pose_textbox.config(text=f"Waiting for Transform ({self.target_frame} -> {self.source_frame})...")
            self.last_pose = None

    def start_recording(self):
        if self.last_pose is None:
            messagebox.showwarning("Warning", "TF data not yet available.")
            return

        self.recording = True
        self.route_data["waypoints"] = {}
        self.waypoint_counter = 0

        # Log the first waypoint (Start Point)
        self.add_waypoint()

        self.status_label.config(text="Status: Recording...", fg="red")
        self.start_button.config(state=tk.DISABLED)
        self.log_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.NORMAL)
        self.get_logger().info("Started recording route.")

    def log_manual_waypoint(self):
        if not self.recording:
            return
        self.add_waypoint()

    def periodic_log_callback(self):
        if self.recording:
            self.add_waypoint()

    def add_waypoint(self):
        if self.last_pose is None:
            return
            
        wp_name = f"waypoint{self.waypoint_counter}"
        
        # Save identical to the requested YAML format
        self.route_data["waypoints"][wp_name] = {
            "pose": [self.last_pose["pose"][0], self.last_pose["pose"][1], 0.0],
            "orientation": self.last_pose["orientation"]
        }
        
        self.waypoint_counter += 1
        self.get_logger().info(f"Logged {wp_name}.")

    def stop_recording(self):
        if not self.recording:
            return

        self.recording = False
        
        # Log the final waypoint
        if self.last_pose is not None:
            self.add_waypoint()

        self.save_to_file()

        self.status_label.config(text="Status: Idle (Saved)", fg="blue")
        self.start_button.config(state=tk.NORMAL)
        self.log_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.DISABLED)
        self.get_logger().info("Stopped recording and saved route.")

    def save_to_file(self):
        try:
            # If the default path was meant to be used, use the name from the entry
            if "route" in self.logging_file_path:
                route_name = self.name_entry.get().strip()
                if not route_name:
                    route_name = "route_obs"
                
                directory = os.path.dirname(self.logging_file_path)
                final_path = os.path.join(directory, f"{route_name}.yaml")
            else:
                final_path = self.logging_file_path

            os.makedirs(os.path.dirname(final_path), exist_ok=True)
            
            with open(final_path, 'w') as yaml_file:
                # Use default_flow_style=None so lists are generated properly like ['x', 'y', 'z']
                # or write a custom yaml dumper to match the block style with "-" exactly.
                
                # To guarantee exact block style arrays (arrays with - element in lines):
                yaml.dump(self.route_data, yaml_file, default_flow_style=False, sort_keys=False)
                
            messagebox.showinfo("Success", f"Route saved to:\n{final_path}")
        except Exception as ex:
            messagebox.showerror("Error", f"Failed to save route: {str(ex)}")


def main(args=None):
    rclpy.init(args=args)

    try:
        from ament_index_python.packages import get_package_share_directory
        package_path = os.path.join(os.path.expanduser("~"), "simulador_ws/imiev/src/imiev")
        if not os.path.exists(package_path):
             package_path = os.path.expanduser("~")
        
        default_path = os.path.join(package_path, "rutas", "route_obs.yaml")
    except Exception:
        default_path = os.path.expanduser("~/rutas/route_obs.yaml")

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
