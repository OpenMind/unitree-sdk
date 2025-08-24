import rclpy
import threading
import os
import signal
import subprocess
from rclpy.node import Node
from flask import Flask, jsonify, request
from flask_cors import CORS
from typing import List, Dict, Optional

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
            self.process = subprocess.Popen(cmd)
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

        self.slam_manager = ProcessManager()
        self.nav2_manager = ProcessManager()

        self.maps_directory = os.path.abspath("./maps")
        os.makedirs(self.maps_directory, mode=0o755, exist_ok=True)

        self.app = Flask(__name__)
        CORS(self.app)
        self.register_routes()

        self.api_thread = threading.Thread(target=self.run_flask_app)
        self.api_thread.start()

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
                return jsonify({"status": "success", "message": "SLAM started"}), 200
            else:
                return jsonify({"status": "error", "message": "SLAM is already running"}), 400

        @self.app.route('/stop/slam', methods=['POST'])
        def stop_slam():
            """
            Stop SLAM process.
            """
            if self.slam_manager.stop():
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
            map_yaml = data.get('map_yaml', None)
            if self.nav2_manager.start(launch_file, map_yaml):
                return jsonify({"status": "success", "message": "Nav2 started"}), 200
            else:
                return jsonify({"status": "error", "message": "Nav2 is already running"}), 400

        @self.app.route('/stop/nav2', methods=['POST'])
        def stop_nav2():
            """
            Stop Nav2 process.
            """
            if self.nav2_manager.stop():
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
            return jsonify({"slam_status": slam_status, "nav2_status": nav2_status}), 200

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
            result1 = subprocess.run(cmd_posegraph, capture_output=True, text=True, timeout=30)

            if "result=0" in result1.stdout.lower():
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

def main(args=None):
    """
    Main function to initialize the ROS2 node and spin it.
    """
    rclpy.init(args=args)

    orchestrator_api = OrchestratorAPI()

    try:
        rclpy.spin(orchestrator_api)
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator_api.slam_manager.stop()
        orchestrator_api.nav2_manager.stop()
        orchestrator_api.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
