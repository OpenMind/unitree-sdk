import rclpy
import threading
import json
import os
import signal
import subprocess
import time
from rclpy.node import Node
from flask import Flask, jsonify, request
from flask_cors import CORS
from typing import Dict, Optional
from om_api.msg import MapStorage, OMAPIRequest, OMAPIResponse, OMAIRequest, OMAIReponse, OMModeRequest, OMModeReponse
from unitree_go.msg import LowState
import requests
from pydantic import BaseModel, Field
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import Twist
from datetime import datetime

class PositionModel(BaseModel):
    x: float = Field(..., description="X coordinate")
    y: float = Field(..., description="Y coordinate")
    z: float = Field(..., description="Z coordinate")

class OrientationModel(BaseModel):
    x: float = Field(..., description="X component of orientation")
    y: float = Field(..., description="Y component of orientation")
    z: float = Field(..., description="Z component of orientation")
    w: float = Field(..., description="W component of orientation")

class PoseModel(BaseModel):
    position: PositionModel = Field(..., description="Position with x, y, z coordinates")
    orientation: OrientationModel = Field(..., description="Orientation with x, y, z, w components")

class LocationModel(BaseModel):
    name: str = Field(..., description="Name of the location")
    description: Optional[str] = Field(None, description="Description of the location")
    timestamp: Optional[str] = Field(None, description="Timestamp of the location")
    pose: PoseModel = Field(..., description="Pose of the location")

class ProcessManager:
    """
    ProcessManager handles starting and stopping of ROS2 launch files as subprocesses.
    """
    def __init__(self):
        """
        Initialize the ProcessManager with no active process.
        """
        self.process: Optional[subprocess.Popen] = None

    def start(self, launch_file: str, map_yaml: Optional[str] = None) -> bool:
        """
        Start a ROS2 launch file as a subprocess.

        Parameters:
        ----------
        launch_file : str
            The name of the launch file to run (e.g., 'slam_launch.py' or
            'nav2_launch.py').
        map_yaml : Optional[str]
            The path to the map YAML file, if applicable.

        Returns:
        -------
        bool
            True if the process was started successfully, False if a process is
            already running.
        """
        if self.process is None or self.process.poll() is not None:
            cmd = [
                'ros2',
                'launch',
                'go2_sdk',
                launch_file
            ]
            if map_yaml:
                cmd.append(f'map_yaml_file:={map_yaml}')
            self.process = subprocess.Popen(cmd, preexec_fn=os.setsid)
            return True
        return False

    def stop(self) -> bool:
        """
        Stop the currently running subprocess.

        Returns:
        -------
        bool
            True if the process was stopped successfully, False if no process
            was running.
        """
        if self.process and self.process.poll() is None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                    self.process.wait()
                return True
            except (ProcessLookupError, OSError) as e:
                try:
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait()
                    return True
                except (ProcessLookupError, OSError):
                    return True
        return False

    def is_running(self) -> bool:
        """
        Check if the process is currently running.

        Returns:
        -------
        bool
            True if process is running, False otherwise.
        """
        return self.process is not None and self.process.poll() is None


class OrchestratorAPI(Node):
    """
    OrchestratorAPI is a ROS2 node that serves as an interface for orchestrating
    various components of the robot system.
    """
    def __init__(self):
        super().__init__('orchestrator_api')

        self.base_control_manager = ProcessManager()
        self.slam_manager = ProcessManager()
        self.nav2_manager = ProcessManager()
        self.charging_manager = ProcessManager()

        self.map_saver_publisher = self.create_publisher(MapStorage, '/om/map_storage', 10)

        self.maps_directory = os.path.abspath("./maps")
        self.locations_directory = os.path.abspath("./locations")

        os.makedirs(self.maps_directory, mode=0o755, exist_ok=True)
        os.makedirs(self.locations_directory, mode=0o755, exist_ok=True)

        # Subscribe to lowstate for charging detection
        self.lowstate_sub = self.create_subscription(
            LowState,
            '/lf/lowstate',
            self.lowstate_callback,
            10
        )

        # Charging state tracking
        self.is_charging = False
        self.battery_soc = 0.0
        self.battery_current = 0.0
        self.charging_confirmed_time = None
        self.charging_confirmation_duration = 3.0  # Wait 3 seconds to confirm stable charging

        # TF2 setup for getting robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.app = Flask(__name__)
        CORS(self.app)
        self.register_routes()

        self.api_thread = threading.Thread(target=self.run_flask_app)
        self.api_thread.start()

        self.api_request_sub = self.create_subscription(
            OMAPIRequest,
            '/om/api/request',
            self.api_request_callback,
            10,
        )

        self.api_response_pub = self.create_publisher(
            OMAPIResponse,
            '/om/api/response',
            10,
        )

        self.ai_request_pub = self.create_publisher(
            OMAIRequest,
            '/om/ai/request',
            10,
        )

        self.ai_request_sub = self.create_subscription(
            OMAIReponse,
            '/om/ai/response',
            self.ai_response_callback,
            10,
        )

        self.mode_request_pub = self.create_publisher(
            OMModeRequest,
            '/om/mode/request',
            10,
        )

        self.mode_request_sub = self.create_subscription(
            OMModeReponse,
            '/om/mode/response',
            self.mode_response_callback,
            10,
        )

        self.move_cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10,
        )

        self.get_logger().info("Orchestrator API Node has been started.")

    def lowstate_callback(self, msg):
        """
        Monitor BMS state for charging detection.

        Parameters:
        ----------
        msg : LowState
            The lowstate message containing BMS data.
        """
        bms = msg.bms_state
        self.battery_soc = bms.soc
        self.battery_current = bms.current

        # Detect charging based on current (positive = charging)
        current_is_charging = bms.current > 0.3  # 0.3 mA threshold to avoid noise

        # Handle charging state transition
        if current_is_charging and not self.is_charging:
            # Just started charging - begin confirmation period
            if self.charging_confirmed_time is None:
                self.charging_confirmed_time = time.time()
                self.get_logger().info(f"Charging current detected ({bms.current} mA), confirming...")
            else:
                # Check if we've been charging long enough to confirm
                elapsed = time.time() - self.charging_confirmed_time
                if elapsed >= self.charging_confirmation_duration:
                    self.is_charging = True
                    self.get_logger().info(f"Charging CONFIRMED after {elapsed:.1f}s - docking successful!")

                    # Stop the charging dock process since we're now charging
                    if self.charging_manager.is_running():
                        self.get_logger().info("Stopping charging dock process - robot is now charging")
                        self.charging_manager.stop()

                    self.charging_confirmed_time = None

        elif not current_is_charging and self.is_charging:
            # Stopped charging
            self.is_charging = False
            self.charging_confirmed_time = None
            self.get_logger().info("Charging stopped")

        elif not current_is_charging:
            # Not charging and confirmation timer was running - reset it
            if self.charging_confirmed_time is not None:
                self.get_logger().info("Charging detection cancelled - current dropped")
                self.charging_confirmed_time = None

    def get_robot_pose_in_map(self):
        """
        Get the current robot pose in the map frame.

        Returns:
        -------
        dict or None
            Dictionary with position and orientation, or None if transform fails.
        """
        try:
            # Lookup transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Extract position and orientation
            pose = {
                "position": {
                    "x": transform.transform.translation.x,
                    "y": transform.transform.translation.y,
                    "z": transform.transform.translation.z
                },
                "orientation": {
                    "x": transform.transform.rotation.x,
                    "y": transform.transform.rotation.y,
                    "z": transform.transform.rotation.z,
                    "w": transform.transform.rotation.w
                }
            }

            return pose

        except TransformException as e:
            self.get_logger().error(f"Failed to get transform: {str(e)}")
            return None

    def run_flask_app(self):
        """
        Register the api routes and run the Flask application.
        """
        self.app.run(host='0.0.0.0', port=5000, use_reloader=False)

    def register_routes(self):
        """
        Register the API routes for the Flask application.
        """

        @self.app.route('/start/base_control', methods=['POST'])
        def start_base_control():
            """
            Start base control process.
            """
            if self.is_slam_running() or self.is_nav2_running():
                return jsonify({"status": "error", "message": "Cannot start base control while SLAM or Nav2 is running"}), 400

            data = request.get_json(silent=True) or {}
            launch_file = data.get('launch_file', 'base_control_launch.py')

            if self.base_control_manager.start(launch_file):
                return jsonify({"status": "success", "message": "Base control started"}), 200
            else:
                return jsonify({"status": "error", "message": "Base control is already running"}), 400

        @self.app.route('/stop/base_control', methods=['POST'])
        def stop_base_control():
            """
            Stop base control process.
            """
            if self.base_control_manager.stop():
                return jsonify({"status": "success", "message": "Base control stopped"}), 200
            else:
                return jsonify({"status": "error", "message": "Base control is not running"}), 400

        @self.app.route('/start/slam', methods=['POST'])
        def start_slam():
            """
            Start SLAM process if Nav2 is not running.
            """
            if self.nav2_manager.process and self.nav2_manager.process.poll() is None:
                return jsonify({"status": "Nav2 is running, stop it before starting SLAM"}), 400

            data = request.get_json(silent=True) or {}
            launch_file = data.get('launch_file', 'slam_launch.py')
            map_yaml = data.get('map_yaml', None)
            if self.slam_manager.start(launch_file, map_yaml):
                # Clear all existing location files for fresh SLAM data collection
                self.clear_all_location_files()
                self.manage_base_control()
                return jsonify({"status": "success", "message": "SLAM started"}), 200
            else:
                return jsonify({"status": "error", "message": "SLAM is already running"}), 400

        @self.app.route('/stop/slam', methods=['POST'])
        def stop_slam():
            """
            Stop SLAM process.
            """
            if self.slam_manager.stop():
                self.manage_base_control()
                return jsonify({"status": "success", "message": "SLAM stopped"}), 200
            else:
                return jsonify({"status": "success", "message": "SLAM is not running"}), 400

        @self.app.route('/start/nav2', methods=['POST'])
        def start_nav2():
            """
            Start Nav2 process if SLAM is not running.
            """
            if self.slam_manager.process and self.slam_manager.process.poll() is None:
                return jsonify({"status": "error", "message": "SLAM is running, stop it before starting Nav2"}), 400

            data = request.get_json(silent=True) or {}
            launch_file = data.get('launch_file', 'nav2_launch.py')
            map_name = data.get('map_name', None)
            if not map_name:
                return jsonify({"status": "error", "message": "map_name is required to start Nav2"}), 400

            map_yaml = os.path.join(self.maps_directory, map_name, f"{map_name}.yaml")
            if self.nav2_manager.start(launch_file, map_yaml):
                self.manage_base_control()

                map_locations_file = os.path.join(self.maps_directory, map_name, 'locations.json')
                locations_file = os.path.join(self.locations_directory, 'locations.json')

                os.makedirs(os.path.dirname(map_locations_file), mode=0o755, exist_ok=True)
                os.makedirs(os.path.dirname(locations_file), mode=0o755, exist_ok=True)

                existing_locations = {}
                if os.path.exists(map_locations_file):
                    try:
                        with open(map_locations_file, 'r') as f:
                            existing_locations = json.load(f)
                    except Exception as e:
                        self.get_logger().error(f"The locations file for map {map_name} is corrupted: {str(e)}")

                with open(map_locations_file, 'w') as f:
                    json.dump(existing_locations, f, indent=4)

                with open(locations_file, 'w') as f:
                    json.dump(existing_locations, f, indent=4)

                return jsonify({"status": "success", "message": "Nav2 started"}), 200
            else:
                return jsonify({"status": "error", "message": "Nav2 is already running"}), 400

        @self.app.route('/stop/nav2', methods=['POST'])
        def stop_nav2():
            """
            Stop Nav2 process.
            """
            if self.nav2_manager.stop():
                self.manage_base_control()
                return jsonify({"status": "success", "message": "Nav2 stopped"}), 200
            else:
                return jsonify({"status": "error", "message": "Nav2 is not running"}), 400

        @self.app.route('/charging/dock', methods=['POST'])
        def start_charging_dock():
            """
            Start autonomous docking sequence to charger.
            Requires Nav2 to be running.
            """
            # Check if Nav2 is running (prerequisite)
            if not self.is_nav2_running():
                return jsonify({
                    "status": "error",
                    "message": "Nav2 must be running to start charging dock. Please start Nav2 first."
                }), 400

            # Check if charging process is already running
            if self.charging_manager.is_running():
                return jsonify({
                    "status": "error",
                    "message": "Charging dock process is already running"
                }), 400

            # Check if already charging
            if self.is_charging:
                return jsonify({
                    "status": "error",
                    "message": "Robot is already charging"
                }), 400

            # Start the charging dock process
            data = request.get_json(silent=True) or {}
            launch_file = data.get('launch_file', 'go2_charge.launch.py')

            if self.charging_manager.start(launch_file):
                self.get_logger().info("Starting autonomous docking to charger")
                return jsonify({
                    "status": "success",
                    "message": "Charging dock process started"
                }), 200
            else:
                return jsonify({
                    "status": "error",
                    "message": "Failed to start charging dock process"
                }), 500

        @self.app.route('/charging/stop', methods=['POST'])
        def stop_charging_dock():
            """
            Stop the charging dock process.
            """
            if self.charging_manager.stop():
                self.charging_confirmed_time = None  # Reset confirmation timer
                self.get_logger().info("Charging dock process stopped")
                return jsonify({
                    "status": "success",
                    "message": "Charging dock process stopped"
                }), 200
            else:
                return jsonify({
                    "status": "error",
                    "message": "Charging dock process is not running"
                }), 400

        @self.app.route('/charging/status', methods=['GET'])
        def charging_status():
            """
            Get current charging and docking status.
            """
            return jsonify({
                "is_charging": self.is_charging,
                "battery_soc": self.battery_soc,
                "battery_current": self.battery_current,
                "dock_process_running": self.charging_manager.is_running(),
                "charging_confirmation_pending": self.charging_confirmed_time is not None
            }), 200

        @self.app.route('/status', methods=['GET'])
        def status():
            """
            Get the status of SLAM and Nav2 processes.
            """
            slam_status = "running" if self.slam_manager.process and self.slam_manager.process.poll() is None else "stopped"
            nav2_status = "running" if self.nav2_manager.process and self.nav2_manager.process.poll() is None else "stopped"
            base_control_status = "running" if self.base_control_manager.process and self.base_control_manager.process.poll() is None else "stopped"
            charging_dock_status = "running" if self.charging_manager.is_running() else "stopped"

            status_data = {
                "slam_status": slam_status,
                "nav2_status": nav2_status,
                "base_control_status": base_control_status,
                "charging_dock_status": charging_dock_status,
                "is_charging": self.is_charging,
                "battery_soc": self.battery_soc,
                "battery_current": self.battery_current
            }

            return jsonify({"status": "success", "message": json.dumps(status_data)}), 200

        @self.app.route('/maps/save', methods=['POST'])
        def save_map():
            """
            Save the current map to a specified filename.
            """
            if not (self.slam_manager.process and self.slam_manager.process.poll() is None):
                return jsonify({"status": "error", "message": "SLAM is not running"}), 400

            data = request.get_json(silent=True) or {}
            if not data or 'map_name' not in data:
                return jsonify({"status": "error", "message": "map_name is required"}), 400

            map_name = data['map_name']
            map_directory = data.get('map_directory', None)

            if not map_name or any(char in map_name for char in ['/', '\\', '..', ' ']):
                return jsonify({"status": "error", "message": "Invalid map name"}), 400

            result = self.save_map(map_name, map_directory)

            files_created = result.get("files_created", [])
            base_path = result.get("base_path", "")

            map_storage_msg = MapStorage()
            map_storage_msg.header.stamp = self.get_clock().now().to_msg()
            map_storage_msg.header.frame_id = "om_api"
            map_storage_msg.map_name = map_name
            map_storage_msg.files_created = files_created
            map_storage_msg.base_path = base_path

            self.map_saver_publisher.publish(map_storage_msg)

            if result["status"] == "success":
                return jsonify(result), 200
            else:
                return jsonify(result), 500

        @self.app.route('/maps/list', methods=['GET'])
        def list_maps():
            """
            List all saved maps in the maps directory.
            """
            maps = []
            for root, dirs, files in os.walk(self.maps_directory):
                for dir_name in dirs:
                    dir_path = os.path.join(root, dir_name)
                    map_files = [f for f in os.listdir(dir_path) if f.startswith(dir_name) and (f.endswith('.yaml') or f.endswith('.pgm') or f.endswith('.posegraph') or f.endswith('.data'))]
                    if map_files:
                        maps.append({
                            "map_name": dir_name,
                            "files": map_files,
                            "path": dir_path
                        })
            return jsonify({"maps": maps}), 200

        @self.app.route('/maps/delete', methods=['POST'])
        def delete_map():
            """
            Delete a specified map directory.
            """
            data = request.get_json(silent=True) or {}
            if not data or 'map_name' not in data:
                return jsonify({"status": "error", "message": "map_name is required"}), 400

            map_name = data['map_name']
            if not map_name or any(char in map_name for char in ['/', '\\', '..', ' ']):
                return jsonify({"status": "error", "message": "Invalid map name"}), 400

            map_path = os.path.join(self.maps_directory, map_name)
            if not os.path.exists(map_path) or not os.path.isdir(map_path):
                return jsonify({"status": "error", "message": "Map does not exist"}), 404

            try:
                for root, dirs, files in os.walk(map_path, topdown=False):
                    for file in files:
                        os.remove(os.path.join(root, file))
                    for dir in dirs:
                        os.rmdir(os.path.join(root, dir))
                os.rmdir(map_path)
                return jsonify({"status": "success", "message": f"Map {map_name} deleted"}), 200
            except Exception as e:
                return jsonify({"status": "error", "message": f"Failed to delete map: {str(e)}"}), 500

        @self.app.route('/maps/locations/add', methods=['POST'])
        def add_map_location():
            """
            Add a single location to the existing map locations JSON file.
            """
            data = request.get_json(silent=True) or {}
            if not data or 'location' not in data:
                return jsonify({"status": "error", "message": "location data is required"}), 400

            location_data = data['location']

            try:
                validated_location = LocationModel(**location_data)
                location = validated_location.model_dump()
            except Exception as e:
                return jsonify({"status": "error", "message": f"Invalid location data: {str(e)}"}), 400

            if 'map_name' not in data:
                return jsonify({"status": "error", "message": "map_name is required in data"}), 400
            map_name = data['map_name']

            map_locations_file = os.path.join(self.maps_directory, map_name, 'locations.json')
            locations_file = os.path.join(self.locations_directory, 'locations.json')

            os.makedirs(os.path.dirname(map_locations_file), mode=0o755, exist_ok=True)
            os.makedirs(os.path.dirname(locations_file), mode=0o755, exist_ok=True)

            existing_locations = {}
            if os.path.exists(map_locations_file):
                try:
                    with open(map_locations_file, 'r') as f:
                        existing_locations = json.load(f)
                except Exception as e:
                    return jsonify({"status": "error", "message": f"Failed to read existing locations: {str(e)}"}), 500

            existing_locations[location['name']] = location

            try:
                with open(map_locations_file, 'w') as f:
                    json.dump(existing_locations, f, indent=4)

                with open(locations_file, 'w') as f:
                    json.dump(existing_locations, f, indent=4)

                return jsonify({"status": "success", "message": f"Location '{location['name']}' added/updated"}), 200
            except Exception as e:
                return jsonify({"status": "error", "message": f"Failed to save location: {str(e)}"}), 500

        @self.app.route('/maps/locations/list', methods=['GET'])
        def list_map_locations():
            """
            List all saved map locations.
            """
            locations_file = os.path.join(self.locations_directory, 'locations.json')

            if not os.path.exists(locations_file):
                return jsonify({"locations": []}), 200

            try:
                with open(locations_file, 'r') as f:
                    locations = json.load(f)
                return jsonify({"status": "success", "message": json.dumps(locations)}), 200
            except Exception as e:
                return jsonify({"status": "error", "message": f"Failed to read locations: {str(e)}"}), 500

        @self.app.route('/maps/locations/add/slam', methods=['POST'])
        def add_slam_location():
            """
            Save current robot position during SLAM mode.
            Requires SLAM to be running.
            """
            # Check if SLAM is running
            if not self.is_slam_running():
                return jsonify({
                    "status": "error",
                    "message": "SLAM must be running to save locations"
                }), 400

            data = request.get_json(silent=True) or {}

            # Validate required fields
            if 'map_name' not in data:
                return jsonify({
                    "status": "error",
                    "message": "map_name is required"
                }), 400

            if 'label' not in data:
                return jsonify({
                    "status": "error",
                    "message": "label is required"
                }), 400

            map_name = data['map_name']
            label = data['label']
            description = data.get('description', '')

            # Get current robot pose in map frame
            pose = self.get_robot_pose_in_map()
            if pose is None:
                return jsonify({
                    "status": "error",
                    "message": "Failed to get robot position from map frame"
                }), 500

            # Generate timestamp
            timestamp = datetime.now().isoformat()

            # Create location data
            location = {
                "name": label,
                "description": description,
                "timestamp": timestamp,
                "pose": pose
            }

            # Validate with LocationModel
            try:
                validated_location = LocationModel(**location)
                location = validated_location.model_dump()
            except Exception as e:
                return jsonify({
                    "status": "error",
                    "message": f"Invalid location data: {str(e)}"
                }), 400

            # Create map directory if it doesn't exist
            map_locations_file = os.path.join(self.maps_directory, map_name, 'locations.json')
            locations_file = os.path.join(self.locations_directory, 'locations.json')

            os.makedirs(os.path.dirname(map_locations_file), mode=0o755, exist_ok=True)
            os.makedirs(os.path.dirname(locations_file), mode=0o755, exist_ok=True)

            # Load existing locations for this map
            existing_locations = {}
            if os.path.exists(map_locations_file):
                try:
                    with open(map_locations_file, 'r') as f:
                        existing_locations = json.load(f)
                except Exception as e:
                    self.get_logger().error(f"Failed to read existing locations: {str(e)}")

            # Add/update the new location
            existing_locations[label] = location

            # Save to map-specific locations file
            try:
                with open(map_locations_file, 'w') as f:
                    json.dump(existing_locations, f, indent=4)

                # Also update the global locations file
                with open(locations_file, 'w') as f:
                    json.dump(existing_locations, f, indent=4)

                self.get_logger().info(f"Saved location '{label}' at position ({pose['position']['x']:.2f}, {pose['position']['y']:.2f})")

                return jsonify({
                    "status": "success",
                    "message": f"Location '{label}' saved successfully",
                    "location": location
                }), 200

            except Exception as e:
                return jsonify({
                    "status": "error",
                    "message": f"Failed to save location: {str(e)}"
                }), 500

    def ai_response_callback(self, msg: OMAIReponse):
        """
        Callback for AI response messages.

        Parameters:
        ----------
        msg : OMAIReponse
            The received AI response message.
        """
        response_msg = OMAPIResponse()
        response_msg.header.stamp = self.get_clock().now().to_msg()
        response_msg.request_id = msg.request_id
        response_msg.code = msg.code
        response_msg.status = msg.status
        response_msg.message = msg.status

        self.api_response_pub.publish(response_msg)

    def mode_response_callback(self, msg: OMModeReponse):
        """
        Callback for mode response messages.

        Parameters:
        ----------
        msg : OMModeReponse
            The received mode response message.
        """
        response_msg = OMAPIResponse()
        response_msg.header.stamp = self.get_clock().now().to_msg()
        response_msg.request_id = msg.request_id
        response_msg.code = msg.code
        response_msg.status = msg.current_mode
        response_msg.message = msg.message

        self.api_response_pub.publish(response_msg)

    def save_map(self, map_name: str, map_directory: Optional[str] = None) -> Dict[str, str]:
        """
        Save the current map using both slam_toolbox and standard ROS2 formats.

        Parameters:
        ----------
        map_name : str
            The name to save the map as (without extension).
        map_directory : Optional[str]
            Directory to save the map in. If None, uses default maps directory.

        Returns:
        -------
        Dict[str, str]
            Dictionary containing status and message.
        """
        if map_directory is None:
            map_directory = self.maps_directory

        os.makedirs(map_directory, mode=0o755, exist_ok=True)

        map_folder_path = os.path.join(map_directory, map_name)
        os.makedirs(map_folder_path, mode=0o755, exist_ok=True)

        map_path = os.path.join(map_folder_path, map_name)

        files_created = []
        errors = []

        try:
            cmd_posegraph = [
                "ros2", "service", "call",
                "/slam_toolbox/serialize_map",
                "slam_toolbox/srv/SerializePoseGraph",
                f"{{filename: '{map_path}'}}"
            ]

            self.get_logger().info("Saving pose graph...")
            result = subprocess.run(cmd_posegraph, capture_output=True, text=True, timeout=30)

            if "result=0" in result.stdout.lower():
                posegraph_file = f"{map_path}.posegraph"
                data_file = f"{map_path}.data"

                if os.path.exists(posegraph_file):
                    files_created.append("posegraph")
                if os.path.exists(data_file):
                    files_created.append("data")
            else:
                errors.append("Failed to save posegraph")

        except subprocess.TimeoutExpired:
            errors.append("Posegraph save operation timed out")
        except Exception as e:
            errors.append(f"Posegraph save failed: {str(e)}")

        try:
            cmd = [
                "ros2", "service", "call",
                "/slam_toolbox/save_map",
                "slam_toolbox/srv/SaveMap",
                f"{{name: {{data: '{map_path}'}}}}"
            ]

            self.get_logger().info("Saving YAML map...")
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

            if result.returncode == 0:
                yaml_file = f"{map_path}.yaml"
                pgm_file = f"{map_path}.pgm"

                if os.path.exists(yaml_file):
                    files_created.append("yaml")
                if os.path.exists(pgm_file):
                    files_created.append("pgm")
            else:
                errors.append("Failed to save YAML/PGM files")
                if result.stderr:
                    errors.append(f"YAML save error: {result.stderr}")

        except subprocess.TimeoutExpired:
            errors.append("YAML save operation timed out")
        except Exception as e:
            errors.append(f"YAML save failed: {str(e)}")

        if files_created and not errors:
            return {
                "status": "success",
                "message": f"Map saved successfully as {map_name}",
                "files_created": files_created,
                "base_path": map_path
            }
        elif files_created and errors:
            return {
                "status": "partial_success",
                "message": f"Map partially saved as {map_name}. Some formats failed.",
                "files_created": files_created,
                "base_path": map_path,
                "errors": errors
            }
        else:
            return {
                "status": "error",
                "message": "Map save failed completely",
                "errors": errors
            }

    def is_slam_running(self) -> bool:
        """
        Check if SLAM is currently running.
        """
        return self.slam_manager.process and self.slam_manager.process.poll() is None

    def is_nav2_running(self) -> bool:
        """
        Check if Nav2 is currently running.
        """
        return self.nav2_manager.process and self.nav2_manager.process.poll() is None

    def is_base_control_running(self) -> bool:
        """
        Check if base control is currently running.
        """
        return self.base_control_manager.process and self.base_control_manager.process.poll() is None

    def should_start_base_control(self) -> bool:
        """
        Check if base control should be started (when neither SLAM nor Nav2 is running).
        """
        return not self.is_slam_running() and not self.is_nav2_running()

    def manage_base_control(self):
        """
        Start base control if conditions are met, stop it otherwise.
        """
        if self.should_start_base_control() and not self.is_base_control_running():
            self.base_control_manager.start('base_control_launch.py')
            self.get_logger().info("Base control started automatically")
        elif not self.should_start_base_control() and self.is_base_control_running():
            self.base_control_manager.stop()
            self.get_logger().info("Base control stopped automatically")

    def clear_all_location_files(self):
        """
        Clear all per-map location files when starting SLAM.
        This allows fresh location data to be collected during the new SLAM session.
        """
        try:
            # Clear global locations file
            locations_file = os.path.join(self.locations_directory, 'locations.json')
            if os.path.exists(locations_file):
                with open(locations_file, 'w') as f:
                    json.dump({}, f, indent=4)
                self.get_logger().info("Cleared global locations file")

            # Clear all per-map location files
            cleared_count = 0
            if os.path.exists(self.maps_directory):
                for map_dir in os.listdir(self.maps_directory):
                    map_path = os.path.join(self.maps_directory, map_dir)
                    if os.path.isdir(map_path):
                        locations_file = os.path.join(map_path, 'locations.json')
                        if os.path.exists(locations_file):
                            with open(locations_file, 'w') as f:
                                json.dump({}, f, indent=4)
                            cleared_count += 1
            
            if cleared_count > 0:
                self.get_logger().info(f"Cleared location files for {cleared_count} maps - ready for fresh SLAM data")
            else:
                self.get_logger().info("No existing location files found - starting with clean slate for SLAM")
                
        except Exception as e:
            self.get_logger().error(f"Failed to clear location files: {str(e)}")

    def api_request_callback(self, msg: OMAPIRequest):
        """
        Receive API requests from ROS2 topic and handle them.

        Parameters:
        -----------
        msg : OMAPIRequest
            The incoming API request message.
        """
        threading.Thread(target=self._process_api_request, args=(msg,)).start()

    def _process_api_request(self, msg: OMAPIRequest):
        """
        Process API request in a separate thread.

        Parameters:
        -----------
        msg : OMAPIRequest
            The incoming API request message.
        """
        action = msg.action.lower()
        base_url = "http://localhost:5000"

        try:
            if action == "start_base_control":
                self.get_logger().info("Received request to start base control")
                response = requests.post(f"{base_url}/start/base_control", json={}, timeout=10)

            elif action == "stop_base_control":
                self.get_logger().info("Received request to stop base control")
                response = requests.post(f"{base_url}/stop/base_control", json={}, timeout=10)

            elif action == "start_slam":
                self.get_logger().info("Received request to start SLAM")
                response = requests.post(f"{base_url}/start/slam", json={}, timeout=10)

            elif action == "stop_slam":
                self.get_logger().info("Received request to stop SLAM")
                response = requests.post(f"{base_url}/stop/slam", json={}, timeout=10)

            elif action == "start_nav2":
                self.get_logger().info("Received request to start Nav2")
                data = {"map_name": msg.parameters} if msg.parameters else {}
                response = requests.post(f"{base_url}/start/nav2", json=data, timeout=10)

            elif action == "stop_nav2":
                self.get_logger().info("Received request to stop Nav2")
                response = requests.post(f"{base_url}/stop/nav2", json={}, timeout=10)

            elif action == "start_charging_dock":
                self.get_logger().info("Received request to start charging dock")
                response = requests.post(f"{base_url}/charging/dock", json={}, timeout=10)

            elif action == "stop_charging_dock":
                self.get_logger().info("Received request to stop charging dock")
                response = requests.post(f"{base_url}/charging/stop", json={}, timeout=10)

            elif action == "charging_status":
                self.get_logger().info("Received request for charging status")
                response = requests.get(f"{base_url}/charging/status", timeout=10)

            elif action == "save_map":
                self.get_logger().info("Received request to save map")
                data = {"map_name": msg.parameters} if msg.parameters else {}
                response = requests.post(f"{base_url}/maps/save", json=data, timeout=30)

            elif action == "list_maps":
                self.get_logger().info("Received request to list maps")
                response = requests.get(f"{base_url}/maps/list", timeout=10)

            elif action == "delete_map":
                self.get_logger().info("Received request to delete map")
                data = {"map_name": msg.parameters} if msg.parameters else {}
                response = requests.post(f"{base_url}/maps/delete", json=data, timeout=10)

            elif action == "status":
                self.get_logger().info("Received request for status")
                response = requests.get(f"{base_url}/status", timeout=10)

            elif action == "list_locations":
                self.get_logger().info("Received request to list map locations")
                response = requests.get(f"{base_url}/maps/locations/list", timeout=10)

            elif action == "add_location":
                self.get_logger().info("Received request to add map location")
                try:
                    location_data = json.loads(msg.parameters) if msg.parameters else {}
                except json.JSONDecodeError:
                    raise ValueError("Invalid JSON in parameters")

                if 'map_name' not in location_data:
                    raise ValueError("map_name is required in location data")

                if 'location' not in location_data:
                    raise ValueError("location data is required in location data")

                data = {"map_name": location_data['map_name'], "location": location_data['location']}
                response = requests.post(f"{base_url}/maps/locations/add", json=data, timeout=10)

            elif action == "add_slam_location":
                self.get_logger().info("Received request to add SLAM location")
                try:
                    location_data = json.loads(msg.parameters) if msg.parameters else {}
                except json.JSONDecodeError:
                    raise ValueError("Invalid JSON in parameters")

                if 'map_name' not in location_data:
                    raise ValueError("map_name is required in location data")

                if 'label' not in location_data:
                    raise ValueError("label is required in location data")

                response = requests.post(f"{base_url}/maps/locations/add/slam", json=location_data, timeout=10)

            elif action == "ai_status":
                # Code 2 is for a status request
                self.get_logger().info("Received request for AI status")
                ai_request_msg = OMAIRequest()
                ai_request_msg.header.stamp = self.get_clock().now().to_msg()
                ai_request_msg.request_id = msg.request_id
                ai_request_msg.code = 2
                self.ai_request_pub.publish(ai_request_msg)
                return

            elif action == "enable_ai":
                # Code 1 is for enabling AI
                self.get_logger().info("Received request to enable AI")
                ai_request_msg = OMAIRequest()
                ai_request_msg.header.stamp = self.get_clock().now().to_msg()
                ai_request_msg.header.frame_id = "om_api"
                ai_request_msg.request_id = msg.request_id
                ai_request_msg.code = 1
                self.ai_request_pub.publish(ai_request_msg)
                return

            elif action == "disable_ai":
                # Code 0 is for disabling AI
                self.get_logger().info("Received request to disable AI")
                ai_request_msg = OMAIRequest()
                ai_request_msg.header.stamp = self.get_clock().now().to_msg()
                ai_request_msg.header.frame_id = "om_api"
                ai_request_msg.request_id = msg.request_id
                ai_request_msg.code = 0
                self.ai_request_pub.publish(ai_request_msg)
                return

            elif action == "get_mode":
                # Code 1 is for getting current mode
                self.get_logger().info("Received request to get current mode")
                mode_request_msg = OMModeRequest()
                mode_request_msg.header.stamp = self.get_clock().now().to_msg()
                mode_request_msg.header.frame_id = "om_api"
                mode_request_msg.request_id = msg.request_id
                mode_request_msg.code = 1
                mode_request_msg.mode = ""
                self.mode_request_pub.publish(mode_request_msg)
                return

            elif action == "swicth_mode":
                # Code 0 is for setting mode, parameters should contain the mode string
                if not msg.parameters:
                    raise ValueError("Mode parameter is required for swicth_mode action")
                self.get_logger().info(f"Received request to set mode to {msg.parameters}")
                mode_request_msg = OMModeRequest()
                mode_request_msg.header.stamp = self.get_clock().now().to_msg()
                mode_request_msg.header.frame_id = "om_api"
                mode_request_msg.request_id = msg.request_id
                mode_request_msg.code = 0
                mode_request_msg.mode = msg.parameters
                self.mode_request_pub.publish(mode_request_msg)
                return

            elif action == "remote_control":
                if not msg.parameters:
                    raise ValueError("Command parameter is required for remote_control action")
                try:
                    speed_parameters = json.loads(msg.parameters) if msg.parameters else {}
                except json.JSONDecodeError:
                    raise ValueError("Invalid JSON in parameters")

                vx = speed_parameters.get("vx", 0.0)
                vy = speed_parameters.get("vy", 0.0)
                vyaw = speed_parameters.get("vyaw", 0.0)

                twist = Twist()
                twist.linear.x = float(vx)
                twist.linear.y = float(vy)
                twist.angular.z = float(vyaw)

                self.move_cmd_pub.publish(twist)
                self.get_logger().info(f"Published remote control command: vx={vx}, vy={vy}, vyaw={vyaw}")

                response_msg = OMAPIResponse()
                response_msg.header.stamp = self.get_clock().now().to_msg()
                response_msg.header.frame_id = "om_api"
                response_msg.request_id = msg.request_id
                response_msg.code = 200
                response_msg.status = "success"
                response_msg.message = f"Remote control command published: vx={vx}, vy={vy}, vyaw={vyaw}"
                self.api_response_pub.publish(response_msg)
                return

            else:
                self.get_logger().error(f"Unknown action: {action}")
                response_msg = OMAPIResponse()
                response_msg.header.stamp = self.get_clock().now().to_msg()
                response_msg.header.frame_id = "om_api"
                response_msg.request_id = msg.request_id
                response_msg.code = 400
                response_msg.status = "error"
                response_msg.message = f"Unknown action: {action}"
                self.api_response_pub.publish(response_msg)
                return

            response_msg = OMAPIResponse()
            response_msg.header.stamp = self.get_clock().now().to_msg()
            response_msg.request_id = msg.request_id
            response_msg.code = response.status_code
            response_msg.status = response.json().get("status", "")
            response_msg.message = response.json().get("message", "")

            self.api_response_pub.publish(response_msg)

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"HTTP request failed: {str(e)}")
            response_msg = OMAPIResponse()
            response_msg.request_id = msg.request_id
            response_msg.code = 500
            response_msg.status = "error"
            response_msg.message = str(e)
            self.api_response_pub.publish(response_msg)

def main(args=None):
    """
    Main function to initialize the ROS2 node and spin it.
    """
    rclpy.init(args=args)

    orchestrator_api = OrchestratorAPI()

    try:
        orchestrator_api.manage_base_control()
        rclpy.spin(orchestrator_api)
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator_api.base_control_manager.stop()
        orchestrator_api.slam_manager.stop()
        orchestrator_api.nav2_manager.stop()
        orchestrator_api.charging_manager.stop()
        orchestrator_api.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
