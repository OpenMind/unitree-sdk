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
                return jsonify({"status": "SLAM started"}), 200
            else:
                return jsonify({"status": "SLAM is already running"}), 400

        @self.app.route('/stop/slam', methods=['POST'])
        def stop_slam():
            """
            Stop SLAM process.
            """
            if self.slam_manager.stop():
                return jsonify({"status": "SLAM stopped"}), 200
            else:
                return jsonify({"status": "SLAM is not running"}), 400

        @self.app.route('/start/nav2', methods=['POST'])
        def start_nav2():
            """
            Start Nav2 process if SLAM is not running.
            """
            if self.slam_manager.process and self.slam_manager.process.poll() is None:
                return jsonify({"status": "SLAM is running, stop it before starting Nav2"}), 400

            data = request.json
            launch_file = data.get('launch_file', 'nav2_launch.py')
            map_yaml = data.get('map_yaml', None)
            if self.nav2_manager.start(launch_file, map_yaml):
                return jsonify({"status": "Nav2 started"}), 200
            else:
                return jsonify({"status": "Nav2 is already running"}), 400

        @self.app.route('/stop/nav2', methods=['POST'])
        def stop_nav2():
            """
            Stop Nav2 process.
            """
            if self.nav2_manager.stop():
                return jsonify({"status": "Nav2 stopped"}), 200
            else:
                return jsonify({"status": "Nav2 is not running"}), 400

        @self.app.route('/status', methods=['GET'])
        def status():
            """
            Get the status of SLAM and Nav2 processes.
            """
            slam_status = "running" if self.slam_manager.process and self.slam_manager.process.poll() is None else "stopped"
            nav2_status = "running" if self.nav2_manager.process and self.nav2_manager.process.poll() is None else "stopped"
            return jsonify({"slam_status": slam_status, "nav2_status": nav2_status}), 200

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
