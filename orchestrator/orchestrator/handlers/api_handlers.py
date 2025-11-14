import json
from datetime import datetime
from typing import TYPE_CHECKING

from flask import jsonify, request

if TYPE_CHECKING:
    from ..core.orchestrator_api import OrchestratorAPI

from ..models.data_models import LocationModel


class APIHandlers:
    """
    Handles all Flask API route implementations.
    """

    def __init__(self, orchestrator: "OrchestratorAPI"):
        """
        Initialize the API handlers.

        Parameters:
        -----------
        orchestrator : OrchestratorAPI
            Reference to the main orchestrator instance.
        """
        self.orchestrator = orchestrator

    def register_routes(self, app):
        """
        Register all API routes with the Flask application.

        Parameters:
        -----------
        app : Flask
            The Flask application instance.
        """

        @app.route("/start/base_control", methods=["POST"])
        def start_base_control():
            """
            Start base control process.
            """
            if (
                self.orchestrator.is_slam_running()
                or self.orchestrator.is_nav2_running()
            ):
                return (
                    jsonify(
                        {
                            "status": "error",
                            "message": "Cannot start base control while SLAM or Nav2 is running",
                        }
                    ),
                    400,
                )

            data = request.get_json(silent=True) or {}
            launch_file = data.get("launch_file", "base_control_launch.py")

            if self.orchestrator.base_control_manager.start(launch_file):
                return (
                    jsonify({"status": "success", "message": "Base control started"}),
                    200,
                )
            else:
                return (
                    jsonify(
                        {
                            "status": "error",
                            "message": "Base control is already running",
                        }
                    ),
                    400,
                )

        @app.route("/stop/base_control", methods=["POST"])
        def stop_base_control():
            """
            Stop base control process.
            """
            if self.orchestrator.base_control_manager.stop():
                return (
                    jsonify({"status": "success", "message": "Base control stopped"}),
                    200,
                )
            else:
                return (
                    jsonify(
                        {"status": "error", "message": "Base control is not running"}
                    ),
                    400,
                )

        @app.route("/start/slam", methods=["POST"])
        def start_slam():
            """
            Start SLAM process if Nav2 is not running.
            """
            if (
                self.orchestrator.nav2_manager.process
                and self.orchestrator.nav2_manager.process.poll() is None
            ):
                return (
                    jsonify(
                        {"status": "Nav2 is running, stop it before starting SLAM"}
                    ),
                    400,
                )

            data = request.get_json(silent=True) or {}
            launch_file = data.get("launch_file", "slam_launch.py")
            map_yaml = data.get("map_yaml", None)

            if self.orchestrator.slam_manager.start(launch_file, map_yaml):
                self.orchestrator.location_manager.clear_all_location_files()
                self.orchestrator.manage_base_control()
                return jsonify({"status": "success", "message": "SLAM started"}), 200
            else:
                return (
                    jsonify({"status": "error", "message": "SLAM is already running"}),
                    400,
                )

        @app.route("/stop/slam", methods=["POST"])
        def stop_slam():
            """
            Stop SLAM process.
            """
            if self.orchestrator.slam_manager.stop():
                self.orchestrator.manage_base_control()
                return jsonify({"status": "success", "message": "SLAM stopped"}), 200
            else:
                return (
                    jsonify({"status": "success", "message": "SLAM is not running"}),
                    400,
                )

        @app.route("/start/nav2", methods=["POST"])
        def start_nav2():
            """
            Start Nav2 process if SLAM is not running.
            """
            if (
                self.orchestrator.slam_manager.process
                and self.orchestrator.slam_manager.process.poll() is None
            ):
                return (
                    jsonify(
                        {
                            "status": "error",
                            "message": "SLAM is running, stop it before starting Nav2",
                        }
                    ),
                    400,
                )

            data = request.get_json(silent=True) or {}
            launch_file = data.get("launch_file", "nav2_launch.py")
            map_name = data.get("map_name", None)

            if not map_name:
                return (
                    jsonify(
                        {
                            "status": "error",
                            "message": "map_name is required to start Nav2",
                        }
                    ),
                    400,
                )

            map_yaml = self.orchestrator.map_manager.get_map_yaml_path(map_name)

            if self.orchestrator.nav2_manager.start(launch_file, map_yaml):
                self.orchestrator.manage_base_control()
                # Load existing locations for this map
                self.orchestrator.location_manager.load_map_locations(map_name)
                return jsonify({"status": "success", "message": "Nav2 started"}), 200
            else:
                return (
                    jsonify({"status": "error", "message": "Nav2 is already running"}),
                    400,
                )

        @app.route("/stop/nav2", methods=["POST"])
        def stop_nav2():
            """Stop Nav2 process."""
            if self.orchestrator.nav2_manager.stop():
                self.orchestrator.manage_base_control()
                return jsonify({"status": "success", "message": "Nav2 stopped"}), 200
            else:
                return (
                    jsonify({"status": "error", "message": "Nav2 is not running"}),
                    400,
                )

        @app.route("/charging/dock", methods=["POST"])
        def start_charging_dock():
            """
            Start autonomous docking sequence to charger.
            """
            if not self.orchestrator.is_nav2_running():
                return (
                    jsonify(
                        {
                            "status": "error",
                            "message": "Nav2 must be running to start charging dock. Please start Nav2 first.",
                        }
                    ),
                    400,
                )

            if self.orchestrator.charging_manager.is_dock_process_running():
                return (
                    jsonify(
                        {
                            "status": "error",
                            "message": "Charging dock process is already running",
                        }
                    ),
                    400,
                )

            if self.orchestrator.charging_manager.is_charging:
                return (
                    jsonify(
                        {"status": "error", "message": "Robot is already charging"}
                    ),
                    400,
                )

            data = request.get_json(silent=True) or {}
            launch_file = data.get("launch_file", "go2_charge.launch.py")

            if self.orchestrator.charging_manager.start_dock_sequence(launch_file):
                return (
                    jsonify(
                        {
                            "status": "success",
                            "message": "Charging dock process started",
                        }
                    ),
                    200,
                )
            else:
                return (
                    jsonify(
                        {
                            "status": "error",
                            "message": "Failed to start charging dock process",
                        }
                    ),
                    500,
                )

        @app.route("/charging/stop", methods=["POST"])
        def stop_charging_dock():
            """
            Stop the charging dock process.
            """
            if self.orchestrator.charging_manager.stop_dock_sequence():
                return (
                    jsonify(
                        {
                            "status": "success",
                            "message": "Charging dock process stopped",
                        }
                    ),
                    200,
                )
            else:
                return (
                    jsonify(
                        {
                            "status": "error",
                            "message": "Charging dock process is not running",
                        }
                    ),
                    400,
                )

        @app.route("/charging/status", methods=["GET"])
        def charging_status():
            """
            Get current charging and docking status.
            """
            status = self.orchestrator.charging_manager.get_status()
            return jsonify(status.model_dump()), 200

        @app.route("/status", methods=["GET"])
        def status():
            """
            Get the status of SLAM and Nav2 processes.
            """
            status_data = self.orchestrator.get_process_status()
            return (
                jsonify(
                    {
                        "status": "success",
                        "message": json.dumps(status_data.model_dump()),
                    }
                ),
                200,
            )

        @app.route("/maps/save", methods=["POST"])
        def save_map():
            """
            Save the current map to a specified filename.
            """
            if not self.orchestrator.is_slam_running():
                return (
                    jsonify({"status": "error", "message": "SLAM is not running"}),
                    400,
                )

            data = request.get_json(silent=True) or {}
            if not data or "map_name" not in data:
                return (
                    jsonify({"status": "error", "message": "map_name is required"}),
                    400,
                )

            map_name = data["map_name"]
            map_directory = data.get("map_directory", None)

            if not map_name or any(char in map_name for char in ["/", "\\", "..", " "]):
                return jsonify({"status": "error", "message": "Invalid map name"}), 400

            result = self.orchestrator.map_manager.save_map(map_name, map_directory)

            # Publish map storage message for cloud sync
            self.orchestrator.publish_map_storage_message(result, map_name)

            if result["status"] == "success":
                return jsonify(result), 200
            else:
                return jsonify(result), 500

        @app.route("/maps/list", methods=["GET"])
        def list_maps():
            """
            List all saved maps in the maps directory.
            """
            maps = self.orchestrator.map_manager.list_maps()
            return jsonify({"maps": [map_info.model_dump() for map_info in maps]}), 200

        @app.route("/maps/delete", methods=["POST"])
        def delete_map():
            """
            Delete a specified map directory.
            """
            data = request.get_json(silent=True) or {}
            if not data or "map_name" not in data:
                return (
                    jsonify({"status": "error", "message": "map_name is required"}),
                    400,
                )

            map_name = data["map_name"]
            result = self.orchestrator.map_manager.delete_map(map_name)

            if result["status"] == "success":
                return jsonify(result), 200
            else:
                return (
                    jsonify(result),
                    404 if "does not exist" in result["message"] else 500,
                )

        @app.route("/maps/locations/add", methods=["POST"])
        def add_map_location():
            """Add a single location to the existing map locations JSON file."""
            data = request.get_json(silent=True) or {}
            if not data or "location" not in data:
                return (
                    jsonify(
                        {"status": "error", "message": "location data is required"}
                    ),
                    400,
                )

            location_data = data["location"]

            try:
                validated_location = LocationModel(**location_data)
            except Exception as e:
                return (
                    jsonify(
                        {
                            "status": "error",
                            "message": f"Invalid location data: {str(e)}",
                        }
                    ),
                    400,
                )

            if "map_name" not in data:
                return (
                    jsonify(
                        {"status": "error", "message": "map_name is required in data"}
                    ),
                    400,
                )

            map_name = data["map_name"]
            result = self.orchestrator.location_manager.add_location(
                map_name, validated_location
            )

            if result["status"] == "success":
                return jsonify(result), 200
            else:
                return jsonify(result), 500

        @app.route("/maps/locations/list", methods=["GET"])
        def list_map_locations():
            """
            List all saved map locations.
            """
            locations = self.orchestrator.location_manager.list_locations()
            return (
                jsonify(
                    {
                        "status": "success",
                        "message": json.dumps(
                            {name: loc.model_dump() for name, loc in locations.items()}
                        ),
                    }
                ),
                200,
            )

        @app.route("/maps/locations/add/slam", methods=["POST"])
        def add_slam_location():
            """
            Save current robot position during SLAM mode.
            """
            if not self.orchestrator.is_slam_running():
                return (
                    jsonify(
                        {
                            "status": "error",
                            "message": "SLAM must be running to save locations",
                        }
                    ),
                    400,
                )

            data = request.get_json(silent=True) or {}

            if "map_name" not in data:
                return (
                    jsonify({"status": "error", "message": "map_name is required"}),
                    400,
                )

            if "label" not in data:
                return jsonify({"status": "error", "message": "label is required"}), 400

            map_name = data["map_name"]
            label = data["label"]
            description = data.get("description", "")

            pose = self.orchestrator.transform_service.get_robot_pose_in_map()
            if pose is None:
                return (
                    jsonify(
                        {
                            "status": "error",
                            "message": "Failed to get robot position from map frame",
                        }
                    ),
                    500,
                )

            timestamp = datetime.now().isoformat()

            location_data = {
                "name": label,
                "description": description,
                "timestamp": timestamp,
                "pose": pose,
            }

            try:
                validated_location = LocationModel(**location_data)
            except Exception as e:
                return (
                    jsonify(
                        {
                            "status": "error",
                            "message": f"Invalid location data: {str(e)}",
                        }
                    ),
                    400,
                )

            result = self.orchestrator.location_manager.add_location(
                map_name, validated_location
            )

            if result["status"] == "success":
                if self.orchestrator.get_logger():
                    self.orchestrator.get_logger().info(
                        f"Saved location '{label}' at position ({pose['position']['x']:.2f}, {pose['position']['y']:.2f})"
                    )

                return (
                    jsonify(
                        {
                            "status": "success",
                            "message": f"Location '{label}' saved successfully",
                            "location": validated_location.model_dump(),
                        }
                    ),
                    200,
                )
            else:
                return jsonify(result), 500
