from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import cv2
import numpy as np
from config import Config


@dataclass
class LaneFollowerState:
    DRIVING = "DRIVING"
    LOST = "LOST"


class ImageProcessor:
    def __init__(self, cfg: Config):
        """
        Initializes the ImageProcessor with the application configuration.
        """
        self.cfg = cfg

    def preprocess(self, frame: np.ndarray) -> tuple[np.ndarray, np.ndarray, int, int, np.ndarray]:
        """
        Preprocesses a raw video frame to isolate the lane lines.
        This involves cropping to a region of interest, color-based thresholding,
        and image cleaning operations.
        """
        h, _ = frame.shape[:2]

        # Define the Region of Interest (ROI) where the lane lines are expected.
        y1 = int(h * self.cfg.roi_y_start_frac)
        y2 = int(h * self.cfg.roi_y_end_frac)
        y1 = max(0, min(h - 1, y1))
        y2 = max(y1 + 1, min(h, y2))

        roi = frame[y1:y2].copy()
        
        # Convert to HSV color space to better isolate the dark lines from the white mat.
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        value = hsv[:, :, 2]
        
        # Create a binary image by thresholding the value channel.
        # Pixels with a value less than the threshold are considered part of the black lines.
        # The result is a binary image where the lines are white (255) and the background is black (0).
        binary = ((value < self.cfg.value_threshold) * 255).astype(np.uint8)

        # Apply morphological operations to clean up the binary image.
        # 'MORPH_OPEN' removes small noise specks.
        # 'MORPH_CLOSE' fills in small gaps in the detected lines.
        mk = self.cfg.morph_kernel
        kernel = np.ones((mk, mk), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # Create a combined mask for visualization purposes.
        combined_mask = binary > 0

        return roi, binary, y1, y2, combined_mask

    def find_runs(self, row: np.ndarray) -> list[tuple[int, int]]:
        """
        Finds contiguous runs of white pixels (the detected lines) in a single
        row of the binary image.
        """
        # Find the x-coordinates of all white pixels in the row.
        xs = np.where(row > 0)[0]
        if xs.size == 0:
            return []

        runs: list[tuple[int, int]] = []
        start = int(xs[0])
        prev = int(xs[0])

        # Iterate through the white pixels to group them into continuous runs.
        for x in xs[1:]:
            x = int(x)
            if x != prev + 1:
                # If the run is long enough, save it.
                if prev - start + 1 >= self.cfg.min_black_run_width:
                    runs.append((start, prev))
                start = x
            prev = x

        # Save the last run.
        if prev - start + 1 >= self.cfg.min_black_run_width:
            runs.append((start, prev))

        return runs


class LaneFollower:
    def __init__(self, cfg: Config) -> None:
        """
        Initializes the LaneFollower, which contains the core logic for
        processing frames and managing the robot's state.
        """
        self.cfg = cfg
        self.image_processor = ImageProcessor(cfg)
        self.state = LaneFollowerState.DRIVING
        self.default_line_side = 'right'  # Can be 'left' or 'right'
        self.last_seen_time = 0.0
        self.last_turn_sign = 0
        self.last_target_x: Optional[float] = None
        self.lane_width_est_px = cfg.lane_width_est_px

        # PID controller state variables
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = 0.0
        self.filtered_deriv = 0.0

    def process_frame(self, frame: np.ndarray):
        """
        The main processing function for the lane follower. It takes a video frame,
        finds the lane lines, and calculates the target center of the lane.
        """
        h, w = frame.shape[:2]
        # Preprocess the frame to get a clean binary image.
        roi, binary, y1, y2, combined_mask = self.image_processor.preprocess(frame)
        roi_h, roi_w = binary.shape[:2]
        
        # Create a visualization of the binary image for debugging.
        binary_viz = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        masked_out = ~combined_mask
        binary_viz[masked_out] = (255, 0, 255) # Purple

        # TODO: Think of a better scan method
        # Define the horizontal scan lines within the ROI.
        row_indices = [int(roi_h * frac) for frac in self.cfg.scan_row_fracs]
        row_indices = [max(0, min(roi_h - 1, y)) for y in row_indices]

        row_targets: list[float] = []
        left_pts: list[tuple[int, int]] = []
        right_pts: list[tuple[int, int]] = []

        # First pass: find candidate left and right line points on each scan row.
        rows_info: list[dict] = []
        for ry in row_indices:
            runs = self.image_processor.find_runs(binary[ry])
            
            if not runs:
                rows_info.append({"ry": ry, "left_x": None, "right_x": None})
                continue

            # Sort runs by their horizontal position
            runs.sort(key=lambda r: r[0])
            
            left_x, right_x = None, None

            if len(runs) == 1:
                # If only one run is detected, classify it based on the default_line_side
                run_center = (runs[0][0] + runs[0][1]) / 2.0
                if self.default_line_side == 'right':
                    right_x = run_center
                    left_x = None
                else:
                    left_x = run_center
                    right_x = None
            else:
                # If multiple runs are detected, the rightmost is the right line
                # and the leftmost is the left line.
                left_run = runs[0]
                right_run = runs[-1]
                left_x = (left_run[0] + left_run[1]) / 2.0
                right_x = (right_run[0] + right_run[1]) / 2.0

            rows_info.append({"ry": ry, "left_x": left_x, "right_x": right_x})
            if left_x is not None:
                left_pts.append((int(left_x), int(ry)))
            if right_x is not None:
                right_pts.append((int(right_x), int(ry)))

        # Second pass: calculate the lane center for each scan row.
        for info in rows_info:
            left_x, right_x = info["left_x"], info["right_x"]
            ry = info["ry"]
            target_x = None

            if left_x is not None and right_x is not None:
                # If both lines are found, the target is the midpoint.
                target_x = (left_x + right_x) / 2.0
                lane_w = right_x - left_x
                # Update the estimated lane width using an exponential moving average.
                if self.cfg.min_lane_width_px < lane_w < self.cfg.max_lane_width_px:
                    a = self.cfg.lane_width_update_alpha
                    self.lane_width_est_px = (1 - a) * self.lane_width_est_px + a * lane_w
            elif left_x is not None:
                # If only the left line is found, estimate the center.
                target_x = left_x + self.lane_width_est_px * self.cfg.edge_to_center_factor
            elif right_x is not None:
                # If only the right line is found, estimate the center.
                target_x = right_x - self.lane_width_est_px * self.cfg.edge_to_center_factor

            if target_x is not None:
                row_targets.append(target_x)
                cv2.circle(binary_viz, (int(target_x), ry), 5, (0, 255, 0), -1)

        # If no targets were found on any scan line, the robot is lost.
        if not row_targets:
            return None, binary_viz, y1

        # The final target is the average of the targets from all scan rows.
        target_x = np.mean(row_targets)
        self.last_target_x = target_x
        
        return target_x, binary_viz, y1
