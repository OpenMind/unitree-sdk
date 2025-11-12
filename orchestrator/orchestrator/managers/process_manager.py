"""
Process lifecycle management for ROS2 launch files.
"""
import os
import signal
import subprocess
from typing import Optional


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
            cmd = ["ros2", "launch", "go2_sdk", launch_file]
            if map_yaml:
                cmd.extend(["map_yaml_file:=" + map_yaml])
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
                self.process.wait(timeout=5)
                self.process = None
                return True
            except (ProcessLookupError, OSError, subprocess.TimeoutExpired) as e:
                if self.process:
                    try:
                        os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                    except (ProcessLookupError, OSError):
                        pass
                self.process = None
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
