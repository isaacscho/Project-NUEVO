from __future__ import annotations

import math
import time

import rclpy
from std_srvs.srv import Trigger

from robot import burger_assemblycode as burger
from robot.robot import FirmwareState, Robot, Unit
from robot.util import densify_polyline

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    Motor,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
)


# ---------------------------------------------------------------------------
# Configuration & Hardware Tuning
# ---------------------------------------------------------------------------

POSITION_UNIT = Unit.MM

WHEEL_DIAMETER = 76.2
WHEEL_BASE = 384
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False

RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

VISION_STALE_SEC = 3.0
MIN_DETECTION_CONFIDENCE = 0.50


# ---------------------------------------------------------------------------
# Mission Path Definitions
# ---------------------------------------------------------------------------

KITCHEN_PATH_1 = [
    (0.0, 300.0),
    (0.0, 975.5),
]

KITCHEN_PATH_2 = [
    (0.0, 975.5),
    (0.0, 1115.2),
]

KITCHEN_PATH_3 = [
    (0.0, 1115.2),
    (0.0, 1254.9),
]

# Normal navigation to the LiDAR obstacle section.
# Obstacle avoidance OFF here.
SCAN_PATH_TO_LIDAR_START = [
    (0.0, 1254.9),
    (129.1, 2000.0),
    (129.1, 3600.0),
    (782.1, 3600.0),
    (732.1, 401.6),
    (1636.6, 401.6),
    (1636.6, 700.0)
]

# Only this section uses obstacle avoidance.
LIDAR_OBSTACLE_PATH = [
    (1636.6, 700.0),
    (1636.6, 3300.0),
]

# Continue to customer scan station with obstacle avoidance OFF.
SCAN_PATH_AFTER_LIDAR = [
    (1636.6, 3300.0),
    (1636.6, 3600.0),
    (2088.5, 3600.0),
]


# Stop at prep point
PREP_PATH_CTRL = [
    (2088.5, 3600.0),
    (2541.1, 3600.0),
    (2541.1, 2000.0),
]

# Delivery path 1:
# Run deliver_full_stack(), then drive to final shelf.

CUST_1_FINAL_PATH_CTRL = [
    (2541.1, 2000.0),
    (2670.2, 1500.0),
    (2670.2, 1185.1),
]

STOP_PATH_1 = [
    (2670.2, 1185.1),
    (2541.1, 800.0),
    (2541.1, 603.2),
]

# Delivery path 2:
# Run deliver_full_stack(), then drive to final shelf.
CUST_2_FINAL_PATH_CTRL = [
    (2541.1, 2000.0),
    (2670.2, 1500.0),
    (2670.2, 1045.4),
]

STOP_PATH_2 = [
    (2670.2, 1045.4),
    (2541.1, 800.0),
    (2541.1, 603.2),
]

FINAL_EXIT_PATH = [
    (2541.1, 603.2),
    (2541.1, 0.0),
]


# ---------------------------------------------------------------------------
# Mission Constants
# ---------------------------------------------------------------------------

TRAFFIC_LIGHT_LOOK_DEG = 15.0
TURN_TOLERANCE_DEG = 5.0
STOP_SIGN_LOOK_RIGHT_DEG = -25.0
STOP_SIGN_SCAN_TIMEOUT_SEC = 5.0
STOP_SIGN_PAUSE_SEC = 3.0

LAPF_VELOCITY_MM_S = 60.0
LAPF_TOLERANCE_MM = 50.0
LAPF_MAX_ANGULAR_RAD_S = 1.0
LAPF_LEASH_LENGTH_MM = 400.0
LAPF_REPULSION_RANGE_MM = 400.0
LAPF_TARGET_SPEED_MM_S = 120.0
LAPF_REPULSION_GAIN = 450.0
LAPF_ATTRACTION_GAIN = 1.0
LAPF_FORCE_EMA_ALPHA = 0.35
LAPF_INFLATION_MARGIN_MM = 100.0
LAPF_LEASH_HALF_ANGLE_DEG = 18.0


# ---------------------------------------------------------------------------
# Global Runtime Variables
# ---------------------------------------------------------------------------

identified_customer = None


# ---------------------------------------------------------------------------
# Helpers: Hardware & Path Loading
# ---------------------------------------------------------------------------

def start_robot(robot: Robot) -> None:
    current = robot.get_state()

    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()

    robot.set_state(FirmwareState.RUNNING)


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

    robot.enable_vision()

    robot.enable_lidar()
    robot.set_lidar_mount(
        x_mm=LIDAR_MOUNT_X_MM,
        y_mm=LIDAR_MOUNT_Y_MM,
        theta_deg=LIDAR_MOUNT_THETA_DEG,
    )

    # Front-focused LiDAR filter so it does not constantly avoid side walls.
    robot.set_lidar_filter(
        range_min_mm=150.0,
        range_max_mm=1500.0,
        fov_deg=(-55.0, 55.0),
    )

    robot.start_lidar_world_publisher()


def start_lapf_to_goal(
    robot: Robot,
    goal: tuple[float, float],
):
    return robot.lapf_to_goal(
        goal[0],
        goal[1],
        velocity=LAPF_VELOCITY_MM_S,
        tolerance=LAPF_TOLERANCE_MM,
        leash_length_mm=LAPF_LEASH_LENGTH_MM,
        repulsion_range_mm=LAPF_REPULSION_RANGE_MM,
        target_speed_mm_s=LAPF_TARGET_SPEED_MM_S,
        max_angular_rad_s=LAPF_MAX_ANGULAR_RAD_S,
        repulsion_gain=LAPF_REPULSION_GAIN,
        attraction_gain=LAPF_ATTRACTION_GAIN,
        force_ema_alpha=LAPF_FORCE_EMA_ALPHA,
        inflation_margin_mm=LAPF_INFLATION_MARGIN_MM,
        leash_half_angle_deg=LAPF_LEASH_HALF_ANGLE_DEG,
        blocking=False,
    )


def load_pure_pursuit_path(
    robot: Robot,
    control_points: list[tuple[float, float]],
) -> None:
    path = densify_polyline(control_points, spacing=20.0)

    # Pure pursuit is used only for normal path following.
    # PP obstacle avoidance is disabled here.
    # LAPF handles obstacle avoidance separately in NAV_LAPF_OBSTACLE_SECTION.
    #
    # NOTE:
    # LegacyMixin._nav_follow_pp_path() still requires the obstacle-avoidance
    # tuning arguments even when obstacle_avoidance=False, so they are provided
    # as inactive placeholder values.
    robot._nav_follow_pp_path(
        lookahead_distance=40.0,
        max_linear_speed=120.0,
        max_angular_speed=2.0,
        goal_tolerance=30.0,
        obstacles_range=450.0,
        view_angle=math.radians(90.0),
        safe_dist=300.0,
        avoidance_delay=250,
        offset=320.0,
        alpha_Ld=0.3,
        obstacle_avoidance=False,
    )

    robot.planner.set_path(path)

# ---------------------------------------------------------------------------
# Helpers: Vision & Biometrics
# ---------------------------------------------------------------------------

def check_vision_class(
    robot: Robot,
    class_name: str,
    attribute_key: str | None = None,
    attribute_val: str | None = None,
) -> bool:
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return False

    for detection in robot.get_detections(class_name):
        if float(detection.get("confidence", 0.0)) < MIN_DETECTION_CONFIDENCE:
            continue

        if attribute_key and attribute_val:
            attributes = detection.get("attributes", {})
            detected_value = attributes.get(attribute_key, {}).get("value")

            if detected_value != attribute_val:
                continue

        return True

    return False


def capture_and_encode_face(robot: Robot) -> bool:
    global identified_customer

    print("[VISION] Requesting customer classification from vision_node...")

    client = robot._node.create_client(Trigger, "/vision/capture_target")

    if not client.wait_for_service(timeout_sec=2.0):
        print("[VISION] ERROR: /vision/capture_target service is offline.")
        return False

    request = Trigger.Request()
    future = client.call_async(request)

    deadline = time.monotonic() + 3.0
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)

    if not future.done():
        print("[VISION] ERROR: /vision/capture_target timed out.")
        return False

    result = future.result()

    if result is None:
        print("[VISION] ERROR: /vision/capture_target returned no result.")
        return False

    if not result.success:
        print(f"[VISION] FAILED: {result.message}")
        return False

    identified_customer = result.message
    print(f"[VISION] Customer identified as: {identified_customer}")
    return True


# ---------------------------------------------------------------------------
# Master FSM Loop
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"

    stop_sign_scan_started_at = None
    lapf_motion_handle = None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        # -------------------------------------------------------------------
        # Startup / Idle
        # -------------------------------------------------------------------

        if state == "INIT":
            start_robot(robot)

            burger.setup_elevation_stepper(robot)

            robot.reset_odometry()
            robot.wait_for_pose_update(timeout=0.2)

            print("[FSM] INIT complete. Transitioning to IDLE.")
            state = "IDLE"

        elif state == "IDLE":
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 255)

            if robot.was_button_pressed(Button.BTN_1):
                print(f"[FSM] BTN_1 pressed. Robot starting in 3 seconds. Stand clear!")
                time.sleep(3)

                print(f"[FSM] Turning left {TRAFFIC_LIGHT_LOOK_DEG} degrees to view traffic light.")
                robot.turn_by(
                    delta_deg=TRAFFIC_LIGHT_LOOK_DEG,
                    blocking=True,
                    tolerance_deg=TURN_TOLERANCE_DEG,
                )

                robot.stop()
                robot.set_velocity(0.0, 0.0)
                time.sleep(0.1)

                print("[FSM] Raising elevator for shelf clearance before navigating to bottom bun.")
                burger.go_to_height_1(robot)
                burger.clearance_height_up(robot)

                robot.stop()
                robot.set_velocity(0.0, 0.0)
                time.sleep(0.1)

                print("[FSM] Looking at traffic light. Waiting for green.")
                state = "WAITING_FOR_GREEN"

        # Reset odometry AFTER turning back so pose is clean at
        # (0, 0, 90deg) and matches the physical robot alignment.
        # KITCHEN_PATH_1 starts ahead of the reset pose so pure pursuit
        # does not try to backtrack to the origin behind the robot.
        elif state == "WAITING_FOR_GREEN":
            if check_vision_class(robot, "traffic light", "color", "green"):
                print("[FSM] Green light detected. Turning back straight.")

                robot.turn_by(
                    delta_deg=-TRAFFIC_LIGHT_LOOK_DEG,
                    blocking=True,
                    tolerance_deg=TURN_TOLERANCE_DEG,
                )

                robot.stop()
                robot.set_velocity(0.0, 0.0)
                time.sleep(0.3)

                robot.reset_odometry()
                robot.wait_for_pose_update(timeout=1.0)

                robot.stop()
                robot.set_velocity(0.0, 0.0)
                time.sleep(0.2)

                print("[FSM] Starting kitchen path 1.")
                load_pure_pursuit_path(robot, KITCHEN_PATH_1)
                state = "NAV_KITCHEN_1"

        # -------------------------------------------------------------------
        # Kitchen Sequence 1: Bottom Bun
        # -------------------------------------------------------------------

        elif state == "NAV_KITCHEN_1":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Arrived at bottom bun station. Running bottom bun sequence.")

                robot.stop()
                burger.BottomBun_sequence(robot)

                print("[FSM] Bottom bun sequence complete. Loading kitchen path 2.")
                load_pure_pursuit_path(
                    robot,
                    KITCHEN_PATH_2,
                )
                state = "NAV_KITCHEN_2"

        # -------------------------------------------------------------------
        # Kitchen Sequence 2: Patty
        # -------------------------------------------------------------------

        elif state == "NAV_KITCHEN_2":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Arrived at patty station. Running patty sequence.")

                robot.stop()
                burger.Patty_sequence(robot)

                print("[FSM] Patty sequence complete. Loading kitchen path 3.")
                load_pure_pursuit_path(
                    robot,
                    KITCHEN_PATH_3,
                )
                state = "NAV_KITCHEN_3"

        # -------------------------------------------------------------------
        # Kitchen Sequence 3: Top Bun
        # -------------------------------------------------------------------

        elif state == "NAV_KITCHEN_3":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Arrived at top bun station. Running top bun sequence.")

                robot.stop()
                burger.TopBun_sequence(robot)

                print("[FSM] Burger fully assembled. Navigating to LiDAR obstacle section start.")
                load_pure_pursuit_path(
                    robot,
                    SCAN_PATH_TO_LIDAR_START,
                )
                state = "NAV_TO_LIDAR_START"

        # -------------------------------------------------------------------
        # LiDAR Obstacle Section
        # -------------------------------------------------------------------

        elif state == "NAV_TO_LIDAR_START":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Reached LiDAR obstacle section. Starting LAPF obstacle avoidance.")

                robot.stop()
                robot.set_velocity(0.0, 0.0)
                time.sleep(0.2)

                lapf_motion_handle = start_lapf_to_goal(
                    robot,
                    (1636.6, 3300.0),
                )
                state = "NAV_LAPF_OBSTACLE_SECTION"

        elif state == "NAV_LAPF_OBSTACLE_SECTION":
            if not robot.is_moving():
                print("[FSM] LAPF obstacle section complete. Returning to normal pure pursuit.")

                robot.stop()
                robot.set_velocity(0.0, 0.0)
                time.sleep(0.2)

                lapf_motion_handle = None

                load_pure_pursuit_path(
                    robot,
                    SCAN_PATH_AFTER_LIDAR,
                )
                state = "NAV_TO_CUSTOMER_SCAN"

        # -------------------------------------------------------------------
        # Customer Scan / Route Selection
        # -------------------------------------------------------------------

        elif state == "NAV_TO_CUSTOMER_SCAN":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Scan station reached. Attempting facial recognition.")

                robot.stop()
                state = "MEMORIZING_TARGET"

        elif state == "MEMORIZING_TARGET":
            robot.set_led(LED.BLUE, 255)

            if capture_and_encode_face(robot):
                robot.set_led(LED.BLUE, 0)

                if identified_customer in ("girl.jpg", "guy.jpg"):
                    print(f"[FSM] {identified_customer} identified. Loading shared delivery prep path.")
                    load_pure_pursuit_path(
                        robot,
                        PREP_PATH_CTRL,
                    )
                    state = "NAV_TO_SHARED_DELIVERY_PREP"

                else:
                    print(f"[FSM] Unknown customer label: {identified_customer}. Retrying scan.")
                    time.sleep(0.5)

            else:
                print("[FSM] Failed to identify customer. Retrying scan.")
                time.sleep(0.5)

        # -------------------------------------------------------------------
        # Shared Delivery Prep
        # -------------------------------------------------------------------

        elif state == "NAV_TO_SHARED_DELIVERY_PREP":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Reached shared delivery prep point at (2541.1, 2000.0).")
                print("[FSM] Preparing burger for selected customer.")

                robot.stop()
                burger.deliver_full_stack(robot)

                if identified_customer == "girl.jpg":
                    print("[FSM] Delivery prep complete. Driving to Delivery Spot 1 shelf.")
                    load_pure_pursuit_path(
                        robot,
                        CUST_1_FINAL_PATH_CTRL,
                    )
                    state = "NAV_TO_DELIVERY_SPOT_1"

                elif identified_customer == "guy.jpg":
                    print("[FSM] Delivery prep complete. Driving to Delivery Spot 2 shelf.")
                    load_pure_pursuit_path(
                        robot,
                        CUST_2_FINAL_PATH_CTRL,
                    )
                    state = "NAV_TO_DELIVERY_SPOT_2"

                else:
                    print(f"[FSM] ERROR: Lost customer label at shared prep: {identified_customer}. Retrying scan.")
                    state = "MEMORIZING_TARGET"

        # -------------------------------------------------------------------
        # Final Delivery Navigation / Handoff
        # -------------------------------------------------------------------

        elif state == "NAV_TO_DELIVERY_SPOT_1":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Reached Delivery Spot 1 shelf. Delivering burger to girl.")

                robot.stop()
                burger.deliver_burger_final(robot)

                print("[FSM] Delivered to Spot 1. Driving to stop sign using STOP_PATH_1.")
                load_pure_pursuit_path(
                    robot,
                    STOP_PATH_1,
                )
                state = "NAV_TO_STOP_SIGN"

        elif state == "NAV_TO_DELIVERY_SPOT_2":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Reached Delivery Spot 2 shelf. Delivering burger to guy.")

                robot.stop()
                burger.deliver_burger_final(robot)

                print("[FSM] Delivered to Spot 2. Driving to stop sign using STOP_PATH_2.")
                load_pure_pursuit_path(
                    robot,
                    STOP_PATH_2,
                )
                state = "NAV_TO_STOP_SIGN"

        # -------------------------------------------------------------------
        # Stop Sign / Final Exit
        # -------------------------------------------------------------------

        elif state == "NAV_TO_STOP_SIGN":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Stop sign approach point reached at (2541.1, 603.2).")

                robot.stop()
                robot.set_velocity(0.0, 0.0)
                time.sleep(0.3)

                print("[FSM] Turning right 25 degrees to look for stop sign.")
                robot.turn_by(
                    delta_deg=STOP_SIGN_LOOK_RIGHT_DEG,
                    blocking=True,
                    tolerance_deg=TURN_TOLERANCE_DEG,
                )

                robot.stop()
                robot.set_velocity(0.0, 0.0)
                time.sleep(0.3)

                stop_sign_scan_started_at = time.monotonic()
                state = "LOOK_FOR_STOP_SIGN"

        elif state == "LOOK_FOR_STOP_SIGN":
            stop_sign_seen = check_vision_class(robot, "stop sign")
            scan_timed_out = (
                stop_sign_scan_started_at is not None
                and time.monotonic() - stop_sign_scan_started_at >= STOP_SIGN_SCAN_TIMEOUT_SEC
            )

            if stop_sign_seen or scan_timed_out:
                if stop_sign_seen:
                    print("[FSM] Stop sign detected. Stopping for 3 seconds.")
                else:
                    print("[FSM] Stop-sign scan timeout. Stopping for 3 seconds anyway.")

                robot.stop()
                robot.set_velocity(0.0, 0.0)
                time.sleep(STOP_SIGN_PAUSE_SEC)

                print("[FSM] Turning back forward.")
                robot.turn_by(
                    delta_deg=-STOP_SIGN_LOOK_RIGHT_DEG,
                    blocking=True,
                    tolerance_deg=TURN_TOLERANCE_DEG,
                )

                robot.stop()
                robot.set_velocity(0.0, 0.0)
                time.sleep(0.3)

                print("[FSM] Driving to final exit point.")
                load_pure_pursuit_path(
                    robot,
                    FINAL_EXIT_PATH,
                )
                state = "NAV_TO_FINAL_EXIT"

        elif state == "NAV_TO_FINAL_EXIT":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Final exit point reached. Mission complete.")
                robot.shutdown()
                state = "IDLE"

        # -------------------------------------------------------------------
        # Unknown State Recovery
        # -------------------------------------------------------------------

        else:
            print(f"[FSM] Unknown state: {state}. Returning to IDLE.")
            robot.shutdown()
            state = "IDLE"

        # -------------------------------------------------------------------
        # Emergency Stop Interrupt
        # -------------------------------------------------------------------

        if robot.was_button_pressed(Button.BTN_2):
            print("[FSM] BTN_2 pressed. Emergency stopping robot.")

            if lapf_motion_handle is not None:
                lapf_motion_handle.cancel()
                lapf_motion_handle.wait(timeout=1.0)
                lapf_motion_handle = None

            robot.shutdown()
            state = "IDLE"

        # -------------------------------------------------------------------
        # FSM Refresh Rate Control
        # -------------------------------------------------------------------

        next_tick += period
        sleep_s = next_tick - time.monotonic()

        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()