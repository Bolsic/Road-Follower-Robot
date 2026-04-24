from dataclasses import dataclass


@dataclass
class Config:
    whep_url: str = "http://192.168.1.103:8888/robot"
    drive_base: str = "http://192.168.1.103:8090"

    width: int = 320
    height: int = 240
    flip: bool = False

    # ROI
    roi_y_start_frac: float = 0.48
    roi_y_end_frac: float = 0.95
    img_center_coef: float = 0.5  # 0.5 is the center, < 0.5 is left, > 0.5 is right

    # bottom -> mid -> upper
    scan_row_fracs: tuple[float, float, float] = (0.90, 0.72, 0.52)

    # Value threshold for line detection (only dark pixels can be lines)
    value_threshold: int = 100  # regions with value < this are considered black
    morph_kernel: int = 3
    min_black_run_width: int = 5

    # Lane width estimate for one-side fallback
    lane_width_est_px: float = 220.0
    lane_width_update_alpha: float = 0.18
    min_lane_width_px: float = 110.0
    max_lane_width_px: float = 290.0
    edge_to_center_factor: float = 0.50

    # Control
    base_speed: int = 54
    min_speed: int = 50
    max_speed: int = 60

    kp: float = 0.6
    ki: float = 0.01
    kd: float = 0.5
    deadband_px: int = 8
    max_turn: int = 32

    # Curve anticipation
    preview_gain: float = 1.80
    max_preview_px: float = 45.0

    # Slow in corners
    slow_gain: float = 0.30
    max_slowdown: int = 22

    # Lost handling
    lost_turn_speed: int = 40
    lost_turn_boost: int = 14
    lost_hold_seconds: float = 0.7

    # HTTP
    drive_timeout_s: float = 0.20
    stop_timeout_s: float = 0.20
    drive_resend_interval_s: float = 0.08

    # UI
    show_debug: bool = True
    window_name: str = "Lane Follower Preview"
    binary_window_name: str = "Binary"
