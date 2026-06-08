from __future__ import annotations

import math
import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.robot import FirmwareState, Robot, Unit
from robot.util import densify_polyline


# ---------------------------------------------------------------------------
# Basic Robot Configuration
# ---------------------------------------------------------------------------

POSITION_UNIT = Unit.MM

WHEEL_DIAMETER = 76.2
WHEEL_BASE = 355.6
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False

RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True


# ---------------------------------------------------------------------------
# Simple Full-Course Path
# 22-inch tile scale, bench on left side.
# This is just a driving path. No vision, no burger assembly, no delivery action.
# ---------------------------------------------------------------------------

COURSE_PATH = [
    # Start and kitchen bench-side route

    # Leave kitchen bench and go to scan/customer area
    (0.0, 760.0),
    (0.0, 3673.4),
    (308.8, 3673.4),
    (308.8, 479.4),
    (1397.0, 479.4),
    (1397.0, 3673.4),
    (1676.4, 3673.4),

    # Go to Delivery Spot 1 path area
    (1955.8, 3673.4),
    (1955.8, 700.0),
    (1775.8, 630.0),
    (1775.8, 582.1),

    # Leave delivery area / stop alignment
    (1955.8, 530.0),
    (1955.8, 442.4),
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def start_robot(robot: Robot) -> bool:
    # 1. Check what the state actually is
    current = robot.get_state()
    print(f"[FSM] Initial state check: {current}")

    # 2. If it's in an error state, try to clear it
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        print("[FSM] Error detected. Attempting to reset...")
        robot.reset_estop()
        time.sleep(1.0) # Give it more time to clear internal buffers

    # 3. Request RUNNING state
    robot.set_state(FirmwareState.RUNNING)
    time.sleep(2.0) 

    # 4. Final verification
    current = robot.get_state()
    if current != FirmwareState.RUNNING:
        # Print the specific error code to help identify the hardware fault
        print(f"[FSM] ERROR: Firmware rejected RUNNING. Current state: {current}")
        return False

    print("[FSM] Firmware is now RUNNING.")
    return True


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)

    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )

    # Everything disabled for simple course run.
    robot.enable_vision()
    robot.enable_lidar()


def load_course_path(robot: Robot) -> None:
    path = densify_polyline(COURSE_PATH, spacing=20.0)

    robot._nav_follow_pp_path(
        lookahead_distance=60.0,
        max_linear_speed=180.0,
        max_angular_speed=1.1,
        goal_tolerance=20.0,
        obstacles_range=450.0,
        view_angle=math.radians(70.0),
        safe_dist=250.0,
        avoidance_delay=150,
        alpha_Ld=0.7,
        offset=0.0,
        lane_width=500.0,
        obstacle_avoidance=False,
        x_L=300.0,
    )

    robot.planner.set_path(path)


# ---------------------------------------------------------------------------
# Main FSM
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            if not start_robot(robot):
                print("[FSM] Cannot start robot. Staying IDLE.")
                state = "IDLE"
                continue

            robot.reset_odometry()
            robot.wait_for_pose_update(timeout=0.2)

            print("[FSM] INIT complete. Press BTN_1 to run course.")
            state = "IDLE"

        elif state == "IDLE":
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 255)

            if robot.was_button_pressed(Button.BTN_1):
                print("[FSM] BTN_1 pressed. Loading full course path.")
                load_course_path(robot)
                state = "RUNNING_COURSE"

        elif state == "RUNNING_COURSE":
            nav_status = robot._nav_follow_pp_path_loop()

            if nav_status != "MOVING":
                print("[FSM] Course path complete. Stopping robot.")
                robot.stop()
                robot.shutdown()
                state = "IDLE"

        else:
            print(f"[FSM] Unknown state: {state}. Stopping and returning to IDLE.")
            robot.stop()
            robot.shutdown()
            state = "IDLE"

        if robot.was_button_pressed(Button.BTN_2):
            print("[FSM] BTN_2 pressed. Emergency stop.")
            robot.stop()
            robot.shutdown()
            state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()

        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()