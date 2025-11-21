import os
import subprocess
from typing import Any, Dict, List, Optional

from ..models.data_models import MapInfo


class MapManager:
    """
    Manages map operations including saving, listing, and deleting maps.
    """

    def __init__(self, maps_directory: str, robot_type: str, logger=None):
        """
        Initialize the MapManager.

        Parameters:
        -----------
        maps_directory : str
            Directory where maps are stored.
        robot_type : str
            Type of robot (e.g., "g1", "go2", "default").
        logger
            Logger instance for logging operations.
        """
        self.maps_directory = os.path.abspath(maps_directory)
        self.robot_type = robot_type
        self.logger = logger
        os.makedirs(self.maps_directory, mode=0o755, exist_ok=True)

    def save_map(
        self, map_name: str, map_directory: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Save the current map.
        For G1 robots: Uses RTAB-Map backup only.
        For other robots: Uses slam_toolbox and standard ROS2 formats.

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

        # g1 Robot: Use RTAB-Map backup
        if self.robot_type == "g1":
            try:
                cmd_rtabmap = [
                    "ros2",
                    "service",
                    "call",
                    "/rtabmap/backup",
                    "std_srvs/srv/Empty",
                ]

                if self.logger:
                    self.logger.info("Saving G1 map using RTAB-Map backup...")

                result = subprocess.run(
                    cmd_rtabmap, capture_output=True, text=True, timeout=30
                )

                if result.returncode == 0:
                    if self.logger:
                        self.logger.info("RTAB-Map backup completed successfully")

                    return {
                        "status": "success",
                        "message": "G1 map saved successfully using RTAB-Map backup",
                        "base_path": map_path,
                        "map_name": map_name,
                        "info": "Map saved to RTAB-Map's configured database location",
                    }
                else:
                    return {
                        "status": "error",
                        "message": "RTAB-Map backup failed",
                        "errors": [
                            f"Service call failed with return code {result.returncode}: {result.stderr}"
                        ],
                    }

            except subprocess.TimeoutExpired:
                return {
                    "status": "error",
                    "message": "RTAB-Map backup operation timed out",
                    "errors": ["Operation exceeded 30 second timeout"],
                }
            except Exception as e:
                return {
                    "status": "error",
                    "message": "RTAB-Map backup failed",
                    "errors": [str(e)],
                }

        # Non-g1 Robots: Use existing slam_toolbox methods
        else:
            files_created = []
            errors = []

            # Save pose graph
            try:
                cmd_posegraph = [
                    "ros2",
                    "service",
                    "call",
                    "/slam_toolbox/serialize_map",
                    "slam_toolbox/srv/SerializePoseGraph",
                    f"{{filename: '{map_path}'}}",
                ]

                if self.logger:
                    self.logger.info("Saving pose graph...")
                result = subprocess.run(
                    cmd_posegraph, capture_output=True, text=True, timeout=30
                )

                if "result=0" in result.stdout.lower():
                    posegraph_files = [
                        f
                        for f in os.listdir(map_folder_path)
                        if f.startswith(map_name)
                        and (f.endswith(".posegraph") or f.endswith(".data"))
                    ]
                    if posegraph_files:
                        files_created.extend(posegraph_files)
                        if self.logger:
                            self.logger.info(f"Posegraph saved: {posegraph_files}")
                else:
                    errors.append(f"Posegraph save failed: {result.stderr}")

            except subprocess.TimeoutExpired:
                errors.append("Posegraph save operation timed out")
            except Exception as e:
                errors.append(f"Posegraph save failed: {str(e)}")

            # Save YAML map
            try:
                cmd = [
                    "ros2",
                    "service",
                    "call",
                    "/slam_toolbox/save_map",
                    "slam_toolbox/srv/SaveMap",
                    f"{{name: {{data: '{map_path}'}}}}",
                ]

                if self.logger:
                    self.logger.info("Saving YAML map...")
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

                if result.returncode == 0:
                    yaml_files = [
                        f
                        for f in os.listdir(map_folder_path)
                        if f.startswith(map_name)
                        and (f.endswith(".yaml") or f.endswith(".pgm"))
                    ]
                    if yaml_files:
                        files_created.extend(yaml_files)
                        if self.logger:
                            self.logger.info(f"YAML map saved: {yaml_files}")
                else:
                    errors.append(f"YAML save failed: {result.stderr}")

            except subprocess.TimeoutExpired:
                errors.append("YAML save operation timed out")
            except Exception as e:
                errors.append(f"YAML save failed: {str(e)}")

            if files_created and not errors:
                return {
                    "status": "success",
                    "message": f"Map saved successfully as {map_name}",
                    "files_created": files_created,
                    "base_path": map_path,
                }
            elif files_created and errors:
                return {
                    "status": "partial_success",
                    "message": f"Map partially saved as {map_name}. Some formats failed.",
                    "files_created": files_created,
                    "base_path": map_path,
                    "errors": errors,
                }
            else:
                return {
                    "status": "error",
                    "message": "Map save failed completely",
                    "errors": errors,
                }

    def list_maps(self) -> List[MapInfo]:
        """
        List all saved maps in the maps directory.

        Returns:
        --------
        List[MapInfo]
            List of map information objects.
        """
        maps = []
        for root, dirs, files in os.walk(self.maps_directory):
            for dir_name in dirs:
                dir_path = os.path.join(root, dir_name)
                map_files = [
                    f
                    for f in os.listdir(dir_path)
                    if f.startswith(dir_name)
                    and (
                        f.endswith(".yaml")
                        or f.endswith(".pgm")
                        or f.endswith(".posegraph")
                        or f.endswith(".data")
                    )
                ]
                if map_files:
                    maps.append(
                        MapInfo(map_name=dir_name, files=map_files, path=dir_path)
                    )
        return maps

    def delete_map(self, map_name: str) -> Dict[str, str]:
        """
        Delete a specified map directory.

        Parameters:
        -----------
        map_name : str
            Name of the map to delete.

        Returns:
        --------
        Dict[str, str]
            Dictionary containing status and message.
        """
        if not map_name or any(char in map_name for char in ["/", "\\", "..", " "]):
            return {"status": "error", "message": "Invalid map name"}

        map_path = os.path.join(self.maps_directory, map_name)
        if not os.path.exists(map_path) or not os.path.isdir(map_path):
            return {"status": "error", "message": "Map does not exist"}

        try:
            for root, dirs, files in os.walk(map_path, topdown=False):
                for file in files:
                    os.remove(os.path.join(root, file))
                for dir in dirs:
                    os.rmdir(os.path.join(root, dir))
            os.rmdir(map_path)
            return {"status": "success", "message": f"Map {map_name} deleted"}
        except Exception as e:
            return {"status": "error", "message": f"Failed to delete map: {str(e)}"}

    def get_map_yaml_path(self, map_name: str) -> str:
        """
        Get the path to a map's YAML file.

        Parameters:
        -----------
        map_name : str
            Name of the map.

        Returns:
        --------
        str
            Path to the map's YAML file.
        """
        return os.path.join(self.maps_directory, map_name, f"{map_name}.yaml")
