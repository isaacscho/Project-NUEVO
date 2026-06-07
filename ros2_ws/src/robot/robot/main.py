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
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
)


# ---------------------------------------------------------------------------
# Configuration & Hardware Tuning
# ---------------------------------------------------------------------------

#TAG_ID = 22

POSITION_UNIT = Unit.MM

WHEEL_DIAMETER = 76.2
WHEEL_BASE = 355.6
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = True

RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = False

VISION_STALE_SEC = 3.0
MIN_DETECTION_CONFIDENCE = 0.50


# ---------------------------------------------------------------------------
# Mission Path Definitions
# ---------------------------------------------------------------------------

KITCHEN_PATH_1 = [
    (0.0, 0.0),
    (-180.0, 300.0),
    (-180.0, 372.5),
]

KITCHEN_PATH_2 = [
    (-180.0, 372.5),
    (-180.0, 512.2),
]

KITCHEN_PATH_3 = [
    (-180.0, 512.2),
    (-180.0, 651.9),
]

SCAN_PATH_CTRL = [
    (-180.0, 651.9),
    (0.0, 760.0),
    (0.0, 3073.4),
    (558.8, 3073.4),
    (558.8, 279.4),
    (1397.0, 279.4),
    (1397.0, 3073.4),
    (1676.4, 3073.4),
]

CUST_1_PATH_CTRL = [
    (1676.4, 3073.4),
    (1955.8, 3073.4),
    (1955.8, 700.0),
    (1775.8, 630.0),
    (1775.8, 582.1),
]

CUST_2_PATH_CTRL = [
    (1676.4, 3073.4),
    (1955.8, 3073.4),
    (1955.8, 560.0),
    (1775.8, 500.0),
    (1775.8, 442.4),
]

STOP_PATH_1 = [
    (1775.8, 582.1),
    (1955.8, 530.0),
    (1955.8, 442.4),
]

TRAFFIC_LIGHT_LOOK_DEG = 25.0
TURN_TOLERANCE_DEG = 3.0

POST_DELIVERY_ROLL_SEC = 2.0
STOP_SIGN_LOOK_RIGHT_DEG = -25.0
STOP_SIGN_SCAN_TIMEOUT_SEC = 2.0

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
    robot.set_lidar_filter(
        range_min_mm=LIDAR_RANGE_MIN_MM,
        range_max_mm=LIDAR_RANGE_MAX_MM,
        fov_deg=LIDAR_FOV_DEG,
    )
    robot.start_lidar_world_publisher()

    #robot.set_tracked_tag_id(TAG_ID)


def load_pure_pursuit_path(
    robot: Robot,
    control_points: list[tuple[float, float]],
) -> None:
    path = densify_polyline(control_points, spacing=20.0)

    robot._nav_follow_pp_path(
        lookahead_distance=70.0,
        max_linear_speed=140.0,
        max_angular_speed=1.5,
        goal_tolerance=20.0,
        obstacles_range=450.0,
        view_angle=math.radians(70.0),
        safe_dist=250.0,
        avoidance_delay=150,
        alpha_Ld=0.7,
        offset=270.0,
        lane_width=500.0,
        obstacle_avoidance=True,
        x_L=300.0,
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

    rclpy.spin_until_future_complete(robot._node, future)

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

    post_delivery_roll_started_at = None
    stop_sign_scan_started_at = None

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
            robot._draw_lidar_obstacles()

            if robot.was_button_pressed(Button.BTN_1):
                print("[FSM] BTN_1 pressed. Turning left 25 degrees to view traffic light.")
                robot.turn_by(
                    delta_deg=TRAFFIC_LIGHT_LOOK_DEG,
                    blocking=True,
                    tolerance_deg=TURN_TOLERANCE_DEG,
                )

                print("[FSM] Looking at traffic light. Waiting for green.")
                state = "WAITING_FOR_GREEN"

        elif state == "WAITING_FOR_GREEN":
            if check_vision_class(robot, "traffic light", "color", "green"):
                print("[FSM] Green light detected. Turning back straight.")

                robot.turn_by(
                    delta_deg=-TRAFFIC_LIGHT_LOOK_DEG,
                    blocking=True,
                    tolerance_deg=TURN_TOLERANCE_DEG,
                )

                print("[FSM] Starting kitchen path 1.")
                load_pure_pursuit_path(robot, KITCHEN_PATH_1)
                state = "NAV_KITCHEN_1"

        # -------------------------------------------------------------------
        # Kitchen Sequence 1: Bottom Bun
        # -------------------------------------------------------------------

        elif state == "NAV_KITCHEN_1":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Arrived at bottom bun station. Running bottom bun sequence.")
                burger.BottomBun_sequence(robot)

                print("[FSM] Bottom bun sequence complete. Loading kitchen path 2.")
                load_pure_pursuit_path(robot, KITCHEN_PATH_2)
                state = "NAV_KITCHEN_2"

        # -------------------------------------------------------------------
        # Kitchen Sequence 2: Patty
        # -------------------------------------------------------------------

        elif state == "NAV_KITCHEN_2":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Arrived at patty station. Running patty sequence.")
                burger.Patty_sequence(robot)

                print("[FSM] Patty sequence complete. Loading kitchen path 3.")
                load_pure_pursuit_path(robot, KITCHEN_PATH_3)
                state = "NAV_KITCHEN_3"

        # -------------------------------------------------------------------
        # Kitchen Sequence 3: Top Bun
        # -------------------------------------------------------------------

        elif state == "NAV_KITCHEN_3":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Arrived at top bun station. Running top bun sequence.")
                burger.TopBun_sequence(robot)

                print("[FSM] Burger fully assembled. Navigating to customer scan station.")
                load_pure_pursuit_path(robot, SCAN_PATH_CTRL)
                state = "NAV_TO_CUSTOMER_SCAN"

        # -------------------------------------------------------------------
        # Customer Scan / Route Selection
        # -------------------------------------------------------------------

        elif state == "NAV_TO_CUSTOMER_SCAN":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Scan station reached. Attempting facial recognition.")
                state = "MEMORIZING_TARGET"

        elif state == "MEMORIZING_TARGET":
            robot.set_led(LED.BLUE, 255)

            if capture_and_encode_face(robot):
                robot.set_led(LED.BLUE, 0)

                if identified_customer == "girl.jpg":
                    print("[FSM] Girl identified. Loading path to Delivery Spot 1.")
                    load_pure_pursuit_path(robot, CUST_1_PATH_CTRL)
                    state = "NAV_TO_DELIVERY_SPOT_1"

                elif identified_customer == "guy.jpg":
                    print("[FSM] Guy identified. Loading path to Delivery Spot 2.")
                    load_pure_pursuit_path(robot, CUST_2_PATH_CTRL)
                    state = "NAV_TO_DELIVERY_SPOT_2"

                else:
                    print(f"[FSM] Unknown customer label: {identified_customer}. Retrying scan.")
                    time.sleep(0.5)

            else:
                print("[FSM] Failed to identify customer. Retrying scan.")
                time.sleep(0.5)

        # -------------------------------------------------------------------
        # Delivery Spot Navigation
        # -------------------------------------------------------------------

        elif state == "NAV_TO_DELIVERY_SPOT_1":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Reached Delivery Spot 1. Delivering to girl.")
                state = "DELIVERING_SPOT_1"

        elif state == "NAV_TO_DELIVERY_SPOT_2":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Reached Delivery Spot 2. Delivering to guy.")
                state = "DELIVERING_SPOT_2"

        # -------------------------------------------------------------------
        # Delivery / Handoff
        # -------------------------------------------------------------------

        elif state == "DELIVERING_SPOT_1":
            print("[FSM] Initiating Delivery Spot 1 handoff.")
            burger.deliver_full_stack(robot)

            print("[FSM] Delivered to Spot 1. Loading stop path 1")
            load_pure_pursuit_path(robot, STOP_PATH_1)
            state = "DRIVING_TO_STOP_ALIGNMENT"

        elif state == "DRIVING_TO_STOP_ALIGNMENT":
            nav_status = robot._nav_follow_pp_path_loop()

            if nav_status != "MOVING":
                print("[FSM] STOP_PATH_1 complete. Starting post-delivery roll.")
                robot.stop()
                post_delivery_roll_started_at = None
                state = "POST_DELIVERY_ROLL"

        elif state == "DELIVERING_SPOT_2":
            print("[FSM] Initiating Delivery Spot 2 handoff.")
            burger.deliver_full_stack(robot)

            print("[FSM] Delivered to Spot 2. Starting post-delivery roll.")
            post_delivery_roll_started_at = None
            state = "POST_DELIVERY_ROLL"

        # -------------------------------------------------------------------
        # End Mission
        # -------------------------------------------------------------------

        elif state == "POST_DELIVERY_ROLL":
            if post_delivery_roll_started_at is None:
                post_delivery_roll_started_at = time.monotonic()
                robot.set_velocity(100.0, 0.0)

            if time.monotonic() - post_delivery_roll_started_at >= POST_DELIVERY_ROLL_SEC:
                robot.stop()

                print("[FSM] Post-delivery roll complete. Turning right 25 degrees to look for stop sign.")
                robot.turn_by(
                    delta_deg=STOP_SIGN_LOOK_RIGHT_DEG,
                    blocking=True,
                    tolerance_deg=TURN_TOLERANCE_DEG,
                )

                stop_sign_scan_started_at = time.monotonic()
                state = "LOOK_FOR_STOP_SIGN"

        elif state == "LOOK_FOR_STOP_SIGN":
            if check_vision_class(robot, "stop sign"):
                print("[FSM] Stop sign detected. Halting platform and returning to IDLE.")
                robot.shutdown()
                state = "IDLE"

            elif (
                stop_sign_scan_started_at is not None
                and time.monotonic() - stop_sign_scan_started_at >= STOP_SIGN_SCAN_TIMEOUT_SEC
            ):
                print("[FSM] Stop-sign scan timeout. Halting platform and returning to IDLE.")
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
