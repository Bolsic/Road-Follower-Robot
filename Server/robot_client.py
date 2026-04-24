import requests
from config import Config


def clamp(v: float, lo: int, hi: int) -> int:
    """
    Clamps a value to a given range [lo, hi].
    """
    return max(lo, min(hi, int(round(v))))


class RobotClient:
    def __init__(self, cfg: Config) -> None:
        """
        Initializes the RobotClient with the application configuration.
        This client handles sending commands to the robot's motor controller.
        """
        self.cfg = cfg
        self.session = requests.Session()

    def drive(self, left: int, right: int) -> bool:
        """
        Sends a drive command to the robot with specified left and right motor speeds.
        """
        # Ensure motor speeds are within the valid range.
        left = clamp(left, -255, 255)
        right = clamp(right, -255, 255)
        try:
            # Send the command as an HTTP GET request.
            r = self.session.get(
                f"{self.cfg.drive_base}/drive",
                params={"l": -left, "r": -right},
                timeout=self.cfg.drive_timeout_s,
            )
            return r.ok
        except requests.RequestException:
            # Return False if the request fails (e.g., network error).
            return False

    def stop(self) -> bool:
        """
        Sends a stop command to the robot.
        """
        try:
            r = self.session.get(
                f"{self.cfg.drive_base}/stop",
                timeout=self.cfg.stop_timeout_s,
            )
            return r.ok
        except requests.RequestException:
            return False
