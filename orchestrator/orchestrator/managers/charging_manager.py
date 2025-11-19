import time
from typing import Optional

from ..managers.process_manager import ProcessManager
from ..models.data_models import ChargingStatus


class ChargingManager:
    """
    Manages charging operations and status monitoring.
    """

    def __init__(self, logger=None):
        """
        Initialize the ChargingManager.

        Parameters:
        -----------
        logger
            Logger instance for logging operations.
        """
        self.logger = logger
        self.process_manager = ProcessManager()

        self.is_charging = False
        self.battery_soc = 0.0
        self.battery_current = 0.0
        self.charging_confirmed_time: Optional[float] = None
        self.charging_confirmation_duration = 3.0  # seconds

    def update_battery_state(self, soc: float, current: float):
        """
        Update battery state and detect charging transitions.

        Parameters:
        -----------
        soc : float
            Battery state of charge (0-100).
        current : float
            Battery current in mA.
        """
        self.battery_soc = soc
        self.battery_current = current

        # Detect charging based on current (positive = charging)
        current_is_charging = current > 0.3  # 0.3 mA threshold to avoid noise

        if current_is_charging and not self.is_charging:
            if self.charging_confirmed_time is None:
                self.charging_confirmed_time = time.time()
                if self.logger:
                    self.logger.info("Charging detected, confirming...")
            else:
                # Check if confirmation period has elapsed
                if (
                    time.time() - self.charging_confirmed_time
                    >= self.charging_confirmation_duration
                ):
                    self.is_charging = True
                    self.charging_confirmed_time = None
                    if self.logger:
                        self.logger.info("Charging confirmed")

        elif not current_is_charging and self.is_charging:
            self.is_charging = False
            self.charging_confirmed_time = None
            if self.logger:
                self.logger.info("Charging stopped")

        elif not current_is_charging:
            if self.charging_confirmed_time is not None:
                self.charging_confirmed_time = None

    def start_dock_sequence(self, launch_file: str = "go2_charge.launch.py") -> bool:
        """
        Start autonomous docking sequence to charger.

        Parameters:
        -----------
        launch_file : str
            Launch file to use for charging sequence.

        Returns:
        --------
        bool
            True if dock sequence started successfully, False otherwise.
        """
        if self.process_manager.is_running():
            if self.logger:
                self.logger.warning("Charging dock process is already running")
            return False

        if self.is_charging:
            if self.logger:
                self.logger.warning("Robot is already charging")
            return False

        if self.process_manager.start(launch_file):
            if self.logger:
                self.logger.info("Starting autonomous docking to charger")
            return True
        else:
            if self.logger:
                self.logger.error("Failed to start charging dock process")
            return False

    def stop_dock_sequence(self) -> bool:
        """
        Stop the charging dock process.

        Returns:
        --------
        bool
            True if dock sequence stopped successfully, False otherwise.
        """
        if self.process_manager.stop():
            self.charging_confirmed_time = None  # Reset confirmation timer
            if self.logger:
                self.logger.info("Charging dock process stopped")
            return True
        else:
            if self.logger:
                self.logger.warning("Charging dock process is not running")
            return False

    def get_status(self) -> ChargingStatus:
        """
        Get current charging and docking status.

        Returns:
        --------
        ChargingStatus
            Current charging status information.
        """
        return ChargingStatus(
            is_charging=self.is_charging,
            battery_soc=self.battery_soc,
            battery_current=self.battery_current,
            dock_process_running=self.process_manager.is_running(),
            charging_confirmation_pending=self.charging_confirmed_time is not None,
        )

    def is_dock_process_running(self) -> bool:
        """
        Check if the dock process is currently running.

        Returns:
        --------
        bool
            True if dock process is running, False otherwise.
        """
        return self.process_manager.is_running()
