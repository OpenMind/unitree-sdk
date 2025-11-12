from typing import Optional

from pydantic import BaseModel, Field


class PositionModel(BaseModel):
    """
    3D position coordinates.

    Parameters:
    -----------
    x : float
        X coordinate.
    y : float
        Y coordinate.
    z : float
        Z coordinate.
    """

    x: float = Field(..., description="X coordinate")
    y: float = Field(..., description="Y coordinate")
    z: float = Field(..., description="Z coordinate")


class OrientationModel(BaseModel):
    """
    Quaternion orientation.

    Parameters:
    -----------
    x : float
        X component of orientation.
    y : float
        Y component of orientation.
    z : float
        Z component of orientation.
    w : float
        W component of orientation.
    """

    x: float = Field(..., description="X component of orientation")
    y: float = Field(..., description="Y component of orientation")
    z: float = Field(..., description="Z component of orientation")
    w: float = Field(..., description="W component of orientation")


class PoseModel(BaseModel):
    """
    Complete pose with position and orientation.

    Parameters:
    -----------
    position : PositionModel
        Position with x, y, z coordinates.
    orientation : OrientationModel
        Orientation with x, y, z, w components.
    """

    position: PositionModel = Field(
        ..., description="Position with x, y, z coordinates"
    )
    orientation: OrientationModel = Field(
        ..., description="Orientation with x, y, z, w components"
    )


class LocationModel(BaseModel):
    """
    Named location with pose and metadata.

    Parameters:
    -----------
    name : str
        Name of the location.
    description : Optional[str]
        Description of the location.
    timestamp : Optional[str]
        Timestamp of the location.
    pose : PoseModel
        Pose of the location.
    """

    name: str = Field(..., description="Name of the location")
    description: Optional[str] = Field(None, description="Description of the location")
    timestamp: Optional[str] = Field(None, description="Timestamp of the location")
    pose: PoseModel = Field(..., description="Pose of the location")


class MapInfo(BaseModel):
    """
    Map information.

    Parameters:
    -----------
    map_name : str
        Name of the map.
    files : list[str]
        List of map files.
    path : str
        Path to map directory.
    """

    map_name: str = Field(..., description="Name of the map")
    files: list[str] = Field(..., description="List of map files")
    path: str = Field(..., description="Path to map directory")


class ProcessStatus(BaseModel):
    """
    Process status information.

    Parameters:
    -----------
    slam_status : str
        SLAM process status.
    nav2_status : str
        Nav2 process status.
    base_control_status : str
        Base control process status.
    charging_dock_status : str
        Charging dock process status.
    is_charging : bool
        Whether robot is charging.
    battery_soc : float
        Battery state of charge.
    battery_current : float
        Battery current.
    """

    slam_status: str = Field(..., description="SLAM process status")
    nav2_status: str = Field(..., description="Nav2 process status")
    base_control_status: str = Field(..., description="Base control process status")
    charging_dock_status: str = Field(..., description="Charging dock process status")
    is_charging: bool = Field(..., description="Whether robot is charging")
    battery_soc: float = Field(..., description="Battery state of charge")
    battery_current: float = Field(..., description="Battery current")


class ChargingStatus(BaseModel):
    """
    Charging status information.

    Parameters:
    -----------
    is_charging : bool
        Whether robot is charging.
    battery_soc : float
        Battery state of charge.
    battery_current : float
        Battery current.
    dock_process_running : bool
        Whether dock process is running.
    charging_confirmation_pending : bool
        Whether charging confirmation is pending.
    """

    is_charging: bool = Field(..., description="Whether robot is charging")
    battery_soc: float = Field(..., description="Battery state of charge")
    battery_current: float = Field(..., description="Battery current")
    dock_process_running: bool = Field(
        ..., description="Whether dock process is running"
    )
    charging_confirmation_pending: bool = Field(
        ..., description="Whether charging confirmation is pending"
    )
