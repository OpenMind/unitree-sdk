import rclpy
import threading
import json
import os
import signal
import subprocess
from rclpy.node import Node
from flask import Flask, jsonify, request
from flask_cors import CORS
from typing import Dict, Optional
from om_api.msg import MapStorage, OMAPIRequest, OMAPIResponse
import requests
from pydantic import BaseModel, Field

class PoseModel(BaseModel):
    x: float = Field(..., description="X coordinate")
    y: float = Field(..., description="Y coordinate")
    z: float = Field(..., description="Z coordinate")

class OrientationModel(BaseModel):
    x: float = Field(..., description="X component of orientation")
    y: float = Field(..., description="Y component of orientation")
    z: float = Field(..., description="Z component of orientation")
    w: float = Field(..., description="W component of orientation")

class PoseModel(BaseModel):
    position: PoseModel = Field(..., description="Position with x, y, z coordinates")
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

        self.map_saver_publisher = self.create_publisher(MapStorage, '/om/map_storage', 10)

        self.maps_directory = os.path.abspath("./maps")
        self.locations_directory = os.path.abspath("./locations")

        os.makedirs(self.maps_directory, mode=0o755, exist_ok=True)
        os.makedirs(self.locations_directory, mode=0o755, exist_ok=True)

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

        self.get_logger().info("Orchestrator API Node has been started.")


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

            data = request.json if request.json else {}
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

            data = request.json
            launch_file = data.get('launch_file', 'slam_launch.py')
            map_yaml = data.get('map_yaml', None)
            if self.slam_manager.start(launch_file, map_yaml):
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

            data = request.json
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

                existing_locations = []
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

        @self.app.route('/status', methods=['GET'])
        def status():
            """
            Get the status of SLAM and Nav2 processes.
            """
            slam_status = "running" if self.slam_manager.process and self.slam_manager.process.poll() is None else "stopped"
            nav2_status = "running" if self.nav2_manager.process and self.nav2_manager.process.poll() is None else "stopped"
            base_control_status = "running" if self.base_control_manager.process and self.base_control_manager.process.poll() is None else "stopped"
            return jsonify({"slam_status": slam_status, "nav2_status": nav2_status, "base_control_status": base_control_status}), 200

        @self.app.route('/maps/save', methods=['POST'])
        def save_map():
            """
            Save the current map to a specified filename.
            """
            if not (self.slam_manager.process and self.slam_manager.process.poll() is None):
                return jsonify({"status": "error", "message": "SLAM is not running"}), 400

            data = request.json
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
            data = request.json
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
            data = request.json
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

            existing_locations = []
            if os.path.exists(map_locations_file):
                try:
                    with open(map_locations_file, 'r') as f:
                        existing_locations = json.load(f)
                except Exception as e:
                    return jsonify({"status": "error", "message": f"Failed to read existing locations: {str(e)}"}), 500

            for i, existing_loc in enumerate(existing_locations):
                if existing_loc.get('name') == location['name']:
                    existing_locations[i] = location
                    break
            else:
                existing_locations.append(location)

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
                    location_data = {}
                data = {"location": location_data} if location_data else {}
                response = requests.post(f"{base_url}/maps/locations/add", json=data, timeout=10)

            else:
                self.get_logger().error(f"Unknown action: {action}")
                response_msg = OMAPIResponse()
                response_msg.header.stamp = self.get_clock().now().to_msg()
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
            response_msg.code = response.status_code
            response_msg.status = "error"
            response_msg.message = response.text
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
        orchestrator_api.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
