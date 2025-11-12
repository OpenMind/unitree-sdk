import os
from typing import Dict

import requests

env = os.getenv("ENV", "production")

if env == "development":
    map_api_url = "https://api-dev.openmind.org/api/core/maps/upload"
else:
    map_api_url = "https://api.openmind.org/api/core/maps/upload"


class MapUploadManager:
    """
    Handles uploading maps to cloud storage services.
    """

    def __init__(self, logger):
        """
        Initialize the map upload manager.

        Parameters:
        -----------
        logger : Logger
            Logger instance for logging messages.
        """
        self.logger = logger
        self.api_key = os.getenv("OM_API_KEY")

        if not self.api_key:
            self.logger.error("OM_API_KEY environment variable not set!")

        self.map_api_url = map_api_url

    def upload_map(
        self, map_name: str, files_created: list, base_path: str
    ) -> Dict[str, str]:
        """
        Upload map files to cloud storage.

        Parameters:
        -----------
        map_name : str
            Name of the map to upload.
        files_created : list
            List of file extensions that were created.
        base_path : str
            Base path where map files are located.

        Returns:
        --------
        Dict[str, str]
            Result dictionary with success/error status and message.
        """
        self.logger.info(f"Uploading map: {map_name}")
        self.logger.info(f"Files to upload: {files_created}")
        self.logger.info(f"Base path: {base_path}")

        if not self.api_key:
            error_msg = "Cannot upload map: OM_API_KEY not set."
            self.logger.error(error_msg)
            return {"status": "error", "message": error_msg}

        if not self._validate_map_files(files_created):
            error_msg = "Map files missing: both .yaml and .pgm files are required."
            self.logger.error(error_msg)
            return {"status": "error", "message": error_msg}

        yaml_path = f"{base_path}.yaml"
        pgm_path = f"{base_path}.pgm"

        if not os.path.exists(yaml_path) or not os.path.exists(pgm_path):
            error_msg = "Map files do not exist at the specified paths."
            self.logger.error(error_msg)
            return {"status": "error", "message": error_msg}

        try:
            return self._upload_files(map_name, yaml_path, pgm_path)
        except Exception as e:
            error_msg = f"Unexpected error during upload: {str(e)}"
            self.logger.error(error_msg)
            return {"status": "error", "message": error_msg}

    def _validate_map_files(self, files_created: list) -> bool:
        """
        Validate that required map files are present.

        Parameters:
        -----------
        files_created : list
            List of file extensions that were created.

        Returns:
        --------
        bool
            True if required files are present, False otherwise.
        """
        has_yaml = any("yaml" in file_ext for file_ext in files_created)
        has_pgm = any("pgm" in file_ext for file_ext in files_created)

        return has_yaml and has_pgm

    def _upload_files(
        self, map_name: str, yaml_path: str, pgm_path: str
    ) -> Dict[str, str]:
        """
        Upload the map files to the cloud API.

        Parameters:
        -----------
        map_name : str
            Name of the map.
        yaml_path : str
            Path to the YAML file.
        pgm_path : str
            Path to the PGM file.

        Returns:
        --------
        Dict[str, str]
            Result dictionary with success/error status and message.
        """
        try:
            with open(yaml_path, "rb") as yaml_file, open(pgm_path, "rb") as pgm_file:
                files = {
                    "map_yaml": (f"{map_name}.yaml", yaml_file, "application/x-yaml"),
                    "map_pgm": (
                        f"{map_name}.pgm",
                        pgm_file,
                        "image/x-portable-graymap",
                    ),
                }
                headers = {"Authorization": f"Bearer {self.api_key}"}
                data = {"map_name": map_name}

                self.logger.info(f"Sending upload request to {self.map_api_url}")

                response = requests.put(
                    self.map_api_url,
                    files=files,
                    headers=headers,
                    data=data,
                    timeout=60,  # 60 second timeout for file uploads
                )

                response.raise_for_status()

                success_msg = f"Map '{map_name}' uploaded successfully."
                self.logger.info(success_msg)

                return {
                    "status": "success",
                    "message": success_msg,
                    "response_code": response.status_code,
                    "response_text": response.text,
                }

        except requests.exceptions.Timeout:
            error_msg = f"Upload timeout for map '{map_name}'"
            self.logger.error(error_msg)
            return {"status": "error", "message": error_msg}

        except requests.exceptions.HTTPError as e:
            error_msg = f"HTTP error uploading map '{map_name}': {e}"
            self.logger.error(error_msg)
            return {"status": "error", "message": error_msg}

        except requests.exceptions.RequestException as e:
            error_msg = f"Request error uploading map '{map_name}': {e}"
            self.logger.error(error_msg)
            return {"status": "error", "message": error_msg}

        except FileNotFoundError as e:
            error_msg = f"Map file not found: {e}"
            self.logger.error(error_msg)
            return {"status": "error", "message": error_msg}

        except Exception as e:
            error_msg = f"Failed to upload map '{map_name}': {str(e)}"
            self.logger.error(error_msg)
            return {"status": "error", "message": error_msg}
