import json
import os
from typing import Dict
from ..models.data_models import LocationModel


class LocationManager:
    """
    Manages location operations including saving, listing, and managing locations.
    """

    def __init__(self, maps_directory: str, locations_directory: str, logger=None):
        """
        Initialize the LocationManager.

        Parameters:
        -----------
        maps_directory : str
            Directory where maps are stored.
        locations_directory : str
            Directory where global locations are stored.
        logger
            Logger instance for logging operations.
        """
        self.maps_directory = os.path.abspath(maps_directory)
        self.locations_directory = os.path.abspath(locations_directory)
        self.logger = logger

        os.makedirs(self.maps_directory, mode=0o755, exist_ok=True)
        os.makedirs(self.locations_directory, mode=0o755, exist_ok=True)

    def add_location(self, map_name: str, location: LocationModel) -> Dict[str, str]:
        """
        Add a single location to the existing map locations JSON file.

        Parameters:
        -----------
        map_name : str
            Name of the map to add the location to.
        location : LocationModel
            Location data to add.

        Returns:
        --------
        Dict[str, str]
            Dictionary containing status and message.
        """
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
                return {"status": "error", "message": f"Failed to read existing locations: {str(e)}"}

        location_dict = location.model_dump()
        existing_locations[location.name] = location_dict

        try:
            with open(map_locations_file, 'w') as f:
                json.dump(existing_locations, f, indent=4)

            with open(locations_file, 'w') as f:
                json.dump(existing_locations, f, indent=4)

            return {"status": "success", "message": f"Location '{location.name}' added/updated"}
        except Exception as e:
            return {"status": "error", "message": f"Failed to save location: {str(e)}"}

    def list_locations(self) -> Dict[str, LocationModel]:
        """
        List all saved map locations.

        Returns:
        --------
        Dict[str, LocationModel]
            Dictionary of location name to LocationModel.
        """
        locations_file = os.path.join(self.locations_directory, 'locations.json')

        if not os.path.exists(locations_file):
            return {}

        try:
            with open(locations_file, 'r') as f:
                locations_data = json.load(f)

            locations = {}
            for name, data in locations_data.items():
                try:
                    locations[name] = LocationModel(**data)
                except Exception as e:
                    if self.logger:
                        self.logger.warning(f"Failed to parse location '{name}': {str(e)}")

            return locations
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to read locations: {str(e)}")
            return {}

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
                    json.dump({}, f)

            # Clear all per-map location files
            cleared_count = 0
            if os.path.exists(self.maps_directory):
                for map_dir in os.listdir(self.maps_directory):
                    map_locations_file = os.path.join(self.maps_directory, map_dir, 'locations.json')
                    if os.path.exists(map_locations_file):
                        with open(map_locations_file, 'w') as f:
                            json.dump({}, f)
                        cleared_count += 1

            if cleared_count > 0:
                if self.logger:
                    self.logger.info(f"Cleared location files for {cleared_count} maps")
            else:
                if self.logger:
                    self.logger.info("No location files to clear")

        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to clear location files: {str(e)}")

    def load_map_locations(self, map_name: str) -> Dict[str, LocationModel]:
        """
        Load locations for a specific map and update global locations.

        Parameters:
        -----------
        map_name : str
            Name of the map to load locations for.

        Returns:
        --------
        Dict[str, LocationModel]
            Dictionary of location name to LocationModel.
        """
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
                if self.logger:
                    self.logger.error(f"The locations file for map {map_name} is corrupted: {str(e)}")

        # Ensure both files are in sync
        try:
            with open(map_locations_file, 'w') as f:
                json.dump(existing_locations, f, indent=4)

            with open(locations_file, 'w') as f:
                json.dump(existing_locations, f, indent=4)
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to sync location files: {str(e)}")

        # Convert to LocationModel objects
        locations = {}
        for name, data in existing_locations.items():
            try:
                locations[name] = LocationModel(**data)
            except Exception as e:
                if self.logger:
                    self.logger.warning(f"Failed to parse location '{name}': {str(e)}")

        return locations
