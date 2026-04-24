#!/usr/bin/env python3
import signal
import time
import cv2
import numpy as np
from config import Config
from camera import WHEPReceiver
from robot_client import RobotClient, clamp
from lane_follower import LaneFollower, LaneFollowerState

# Global state to control the main loop
running = True


def handle_sigint(sig, frame):
    """
    Signal handler for Ctrl+C (SIGINT). Sets the global 'running' flag to False
    to gracefully shut down the application.
    """
    global running
    print("SIGINT received, stopping...")
    running = False


def main():
    """
    The main entry point of the application. Initializes all components and
    runs the main control loop.
    """
    global running
    # Register the SIGINT handler
    signal.signal(signal.SIGINT, handle_sigint)

    # Initialize all the major components of the application
    cfg = Config()
    robot = RobotClient(cfg)
    receiver = WHEPReceiver(cfg.whep_url, cfg.width, cfg.height, cfg.flip)
    follower = LaneFollower(cfg)

    # Start the camera feed
    receiver.start()
    print("WHEP receiver started.")

    mode_manual = "MANUAL"
    mode_auto = "AUTO"
    control_mode = cfg.control_mode if cfg.control_mode in (mode_manual, mode_auto) else mode_manual
    manual_left = 0
    manual_right = 0

    # Safety first: ensure stale commands are cleared when controller boots.
    robot.stop()
    print(f"Control mode: {control_mode}")
    print("Keyboard: m=manual n=auto w/s/a/d=move x/space=stop l=toggle-line-side q=quit")

    # Variables for controlling the drive command frequency
    last_drive_time = 0
    last_left = 0
    last_right = 0
    last_no_frame_log_time = 0.0

    try:
        # The main control loop
        while running:
            force_send = False

            key = -1
            if cfg.show_debug:
                key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                running = False
            elif key in (ord('m'), ord('M')):
                if control_mode != mode_manual:
                    control_mode = mode_manual
                    manual_left = 0
                    manual_right = 0
                    follower.integral = 0.0
                    follower.prev_error = 0.0
                    follower.prev_time = 0.0
                    follower.filtered_deriv = 0.0
                    force_send = True
                    print("Control mode set to MANUAL")
            elif key in (ord('n'), ord('N')):
                if control_mode != mode_auto:
                    control_mode = mode_auto
                    follower.integral = 0.0
                    follower.prev_error = 0.0
                    follower.prev_time = 0.0
                    follower.filtered_deriv = 0.0
                    force_send = True
                    print("Control mode set to AUTO")
            elif key in (ord('l'), ord('L')):
                if follower.default_line_side == 'right':
                    follower.default_line_side = 'left'
                    print("Default line side set to LEFT")
                else:
                    follower.default_line_side = 'right'
                    print("Default line side set to RIGHT")
            elif control_mode == mode_manual:
                if key in (ord('w'), ord('W')):
                    manual_left = cfg.manual_speed
                    manual_right = cfg.manual_speed
                    force_send = True
                elif key in (ord('s'), ord('S')):
                    manual_left = -cfg.manual_speed
                    manual_right = -cfg.manual_speed
                    force_send = True
                elif key in (ord('a'), ord('A')):
                    manual_left = -cfg.manual_turn_speed
                    manual_right = cfg.manual_turn_speed
                    force_send = True
                elif key in (ord('d'), ord('D')):
                    manual_left = cfg.manual_turn_speed
                    manual_right = -cfg.manual_turn_speed
                    force_send = True
                elif key in (ord('x'), ord('X'), ord(' ')):
                    manual_left = 0
                    manual_right = 0
                    force_send = True

            if not running:
                break

            # Get the latest frame from the camera
            frame = receiver.get_frame()
            if frame is None:
                now = time.time()
                if now - last_no_frame_log_time > 2.0:
                    state = receiver.get_debug_state()
                    print(
                        "No frame yet | "
                        f"status={state['status']} "
                        f"frames={state['frames_received']} "
                        f"last_frame_age_s={state['last_frame_age_s']} "
                        f"error={state['last_error']} "
                        f"url={state['whep_url']}"
                    )
                    last_no_frame_log_time = now

                if control_mode == mode_manual:
                    left, right = manual_left, manual_right
                else:
                    if cfg.auto_stop_on_no_frame:
                        left, right = 0, 0
                    else:
                        left, right = last_left, last_right

                now = time.time()
                if force_send or (now - last_drive_time > cfg.drive_resend_interval_s):
                    if int(left) == 0 and int(right) == 0:
                        robot.stop()
                    else:
                        robot.drive(int(left), int(right))
                    last_drive_time = now
                    last_left = left
                    last_right = right

                if cfg.show_debug:
                    status_img = np.zeros((240, 640, 3), dtype=np.uint8)
                    cv2.putText(status_img, f"NO FRAME | MODE: {control_mode}", (15, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
                    cv2.putText(status_img, f"CMD L:{int(left)} R:{int(right)}", (15, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(status_img, "m=manual n=auto w/s/a/d move x/space stop q quit", (15, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (190, 190, 190), 1)
                    cv2.imshow(cfg.window_name, status_img)
                time.sleep(0.01)
                continue
            
            h, w = frame.shape[:2]
            img_center = w * cfg.img_center_coef

            # Process the frame to find the lane center
            result, binary_viz, y1 = follower.process_frame(frame)

            if control_mode == mode_auto:
                # State machine: LOST or DRIVING
                if result is None:
                    # --- LOST STATE ---
                    # If we just lost the line, record the time.
                    if follower.state != LaneFollowerState.LOST:
                        follower.state = LaneFollowerState.LOST
                        follower.last_seen_time = time.time()

                    # After a short delay, start turning to try and find the line again.
                    if time.time() - follower.last_seen_time > cfg.lost_hold_seconds:
                        turn = cfg.lost_turn_speed + cfg.lost_turn_boost
                        # Turn in the direction the robot was last turning.
                        if follower.last_turn_sign < 0:
                            left, right = -turn, turn
                        else:
                            left, right = turn, -turn
                    else:
                        # If recently lost, just stop for a moment.
                        left, right = 0, 0
                else:
                    # --- DRIVING STATE ---
                    follower.state = LaneFollowerState.DRIVING
                    target_x = result
                    
                    # --- PID CONTROLLER ---
                    now = time.time()
                    dt = now - follower.prev_time if follower.prev_time > 0 else 1.0 / 30.0
                    follower.prev_time = now

                    # Calculate the error (distance from the center of the image to the target)
                    error = target_x - img_center
                    
                    # Apply a deadband to ignore small errors
                    if abs(error) < cfg.deadband_px:
                        error = 0

                    # Integral term
                    # (accumulates past errors to correct for steady-state error)
                    follower.integral += error * dt
                    follower.integral = clamp(follower.integral, -100, 100)

                    # Derivative term
                    # (predicts future error based on the rate of change)
                    derivative = (error - follower.prev_error) / dt
                    follower.prev_error = error
                    
                    # Apply a simple low-pass filter to the derivative to reduce noise
                    follower.filtered_deriv = 0.7 * follower.filtered_deriv + 0.3 * derivative

                    # Combine the P, I, and D terms to get the final turn command
                    turn = (cfg.kp * error) + (cfg.ki * follower.integral) + (cfg.kd * follower.filtered_deriv)
                    turn = clamp(turn, -cfg.max_turn, cfg.max_turn)

                    # Remember the direction of the last turn
                    if turn != 0:
                        follower.last_turn_sign = np.sign(turn)

                    # --- SPEED CONTROL ---
                    # Slow down in corners for better stability
                    slowdown = int(cfg.slow_gain * abs(turn))
                    slowdown = min(cfg.max_slowdown, slowdown)
                    speed = cfg.base_speed - slowdown
                    speed = clamp(speed, cfg.min_speed, cfg.max_speed)

                    # Calculate final motor speeds
                    left = speed + turn
                    right = speed - turn
            else:
                left, right = manual_left, manual_right

            # Send drive commands to the robot at a fixed interval
            now = time.time()
            if force_send or (now - last_drive_time > cfg.drive_resend_interval_s):
                if int(left) == 0 and int(right) == 0:
                    robot.stop()
                else:
                    robot.drive(int(left), int(right))
                last_drive_time = now
                last_left = left
                last_right = right

            # --- UI AND DEBUGGING ---
            if cfg.show_debug:
                # Draw a line indicating the center of the image
                cv2.line(frame, (int(img_center), 0), (int(img_center), h), (255, 0, 0), 1)
                # Draw a circle at the calculated target position
                if result is not None:
                    cv2.circle(frame, (int(result), y1 + int(binary_viz.shape[0] * 0.9)), 5, (0, 0, 255), -1)

                mode_color = (80, 220, 80) if control_mode == mode_manual else (0, 180, 255)
                cv2.putText(frame, f"MODE: {control_mode}", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, mode_color, 2)
                cv2.putText(frame, f"CMD L:{int(left)} R:{int(right)}", (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, "m=manual n=auto w/s/a/d move x/space stop q quit", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

                # Show the main camera feed and the binary image
                cv2.imshow(cfg.window_name, frame)
                if binary_viz is not None:
                    cv2.imshow(cfg.binary_window_name, binary_viz)

    finally:
        # --- CLEANUP ---
        print("Stopping robot and receiver...")
        robot.stop()
        receiver.stop()
        if cfg.show_debug:
            cv2.destroyAllWindows()
        print("Cleanup complete.")


if __name__ == "__main__":
    main()
