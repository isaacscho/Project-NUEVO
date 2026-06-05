from __future__ import annotations

import math
import time

import rclpy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from robot import burger_assemblycode as burger
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.robot import FirmwareState, Robot, Unit
from robot.util import densify_polyline


# ---------------------------------------------------------------------------
# Configuration & Hardware Tuning
# ---------------------------------------------------------------------------

TAG_ID = 22

POSITION_UNIT = Unit.MM

WHEEL_DIAMETER = 76.2
WHEEL_BASE = 355.6
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

KITCHEN_PATH_1 = [(0.0, 0.0), (0.0, 406.4)]

KITCHEN_PATH_2 = [(0.0, 406.4), (0.0, 558.8)]

KITCHEN_PATH_3 = [(0.0, 558.8), (0.0, 711.2)]

SCAN_PATH_CTRL = [
    (0.0, 711.2),
    (0.0, 3352.8),
    (609.6, 3352.8),
    (609.6, 304.8),
    (1524.0, 304.8),
    (1524.0, 3352.8),
    (1828.8, 3352.8),
]

CUST_1_PATH_CTRL = [
    (1828.8, 3352.8),
    (2133.6, 3352.8),
    (2133.6, 635.0),
]

CUST_2_PATH_CTRL = [
    (2133.6, 635.0),
    (2133.6, 482.6),
]

STOP_PATH_1 = [
    (2133.6, 635.0),
    (2133.6, 0.0),
]

STOP_PATH_2 = [
    (2133.6, 482.6),
    (2133.6, 0.0),
]


# ---------------------------------------------------------------------------
# Global Runtime Variables
# ---------------------------------------------------------------------------

current_vision_match = False
identified_customer = None
vision_match_subscription = None


# ---------------------------------------------------------------------------
# Helpers: Hardware & Path Loading
# ---------------------------------------------------------------------------

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

    robot.set_tracked_tag_id(TAG_ID)


def load_pure_pursuit_path(
    robot: Robot,
    control_points: list[tuple[float, float]],
) -> None:
    path = densify_polyline(control_points, spacing=20.0)

    robot._nav_follow_pp_path(
        lookahead_distance=100.0,
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


def _vision_status_callback(msg: Bool) -> None:
    global current_vision_match
    current_vision_match = bool(msg.data)


def capture_and_encode_face(robot: Robot) -> bool:
    global identified_customer
    global vision_match_subscription

    print("[VISION] Requesting classification from face_tracker...")

    client = robot._node.create_client(Trigger, "/vision/capture_target")

    if not client.wait_for_service(timeout_sec=2.0):
        print("[VISION] ERROR: face_tracker service is offline. Is it running?")
        return False

    request = Trigger.Request()
    future = client.call_async(request)

    rclpy.spin_until_future_complete(robot._node, future)

    result = future.result()

    if result is None:
        print("[VISION] ERROR: face_tracker service returned no result.")
        return False

    if result.success:
        identified_customer = result.message
        print(f"[VISION] Success! Customer identified as: {identified_customer}")

        vision_match_subscription = robot._node.create_subscription(
            Bool,
            "/vision/match_status",
            _vision_status_callback,
            10,
        )

        return True

    print(f"[VISION] FAILED: {result.message}")
    return False


def verify_live_face() -> bool:
    return current_vision_match


# ---------------------------------------------------------------------------
# Master FSM Loop
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        # -------------------------------------------------------------------
        # Startup / Idle
        # -------------------------------------------------------------------

        if state == "INIT":
            robot.set_state(FirmwareState.RUNNING)

            burger.setup_elevation_stepper(robot)

            robot.reset_odometry()
            robot.wait_for_pose_update(timeout=0.2)

            print("[FSM] INIT complete. Transitioning to IDLE.")
            state = "IDLE"

        elif state == "IDLE":
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 255)
            robot._draw_lidar_obstacles()

            if robot.get_button(Button.BTN_1):
                print("[FSM] BTN_1 pressed. Transitioning to WAITING_FOR_GREEN.")
                state = "WAITING_FOR_GREEN"

        elif state == "WAITING_FOR_GREEN":
            if check_vision_class(robot, "traffic light", "color", "green"):
                print("[FSM] Green light detected. Starting kitchen path 1.")
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
        # Customer Scan / Target Capture
        # -------------------------------------------------------------------

        elif state == "NAV_TO_CUSTOMER_SCAN":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Scan station reached. Attempting biometric capture.")
                state = "MEMORIZING_TARGET"

        elif state == "MEMORIZING_TARGET":
            robot.set_led(LED.BLUE, 255)

            if capture_and_encode_face(robot):
                print(f"[FSM] Profile locked: {identified_customer}. Loading Customer 1 path.")
                robot.set_led(LED.BLUE, 0)

                load_pure_pursuit_path(robot, CUST_1_PATH_CTRL)
                state = "NAV_TO_DROPOFF"
            else:
                print("[FSM] Failed to capture target. Retrying.")
                time.sleep(0.5)

        # -------------------------------------------------------------------
        # Customer 1 Verification
        # -------------------------------------------------------------------

        elif state == "NAV_TO_DROPOFF":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Reached Customer 1 drop-off zone. Verifying biometric match.")
                state = "VERIFY_CUST_1"

        elif state == "VERIFY_CUST_1":
            if verify_live_face():
                print("[VISION] MATCH. Target is Customer 1.")
                robot.set_led(LED.GREEN, 255)
                robot.set_led(LED.RED, 0)

                state = "DELIVERING_CUST_1"
            else:
                print("[VISION] NO MATCH. Moving to Customer 2.")
                robot.set_led(LED.RED, 255)
                robot.set_led(LED.GREEN, 0)

                state = "LOAD_CUST_2_PATH"

        # -------------------------------------------------------------------
        # Customer 2 Verification
        # -------------------------------------------------------------------

        elif state == "LOAD_CUST_2_PATH":
            print("[FSM] Loading path to Customer 2.")
            load_pure_pursuit_path(robot, CUST_2_PATH_CTRL)
            state = "NAV_TO_CUST_2"

        elif state == "NAV_TO_CUST_2":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Reached Customer 2 drop-off zone. Verifying biometric match.")
                state = "VERIFY_CUST_2"

        elif state == "VERIFY_CUST_2":
            if verify_live_face():
                print("[VISION] MATCH. Target is Customer 2.")
                robot.set_led(LED.GREEN, 255)
                robot.set_led(LED.RED, 0)

                state = "DELIVERING_CUST_2"
            else:
                print("[FSM] FATAL ERROR: Neither customer matched.")
                robot.shutdown()
                state = "IDLE"

        # -------------------------------------------------------------------
        # Delivery / Handoff
        # -------------------------------------------------------------------

        elif state == "DELIVERING_CUST_1":
            print("[FSM] Initiating Customer 1 handoff.")
            burger.deliver_full_stack(robot)

            print("[FSM] Delivered to Customer 1. Loading stop path 1.")
            load_pure_pursuit_path(robot, STOP_PATH_1)
            state = "DRIVING_TO_STOP"

        elif state == "DELIVERING_CUST_2":
            print("[FSM] Initiating Customer 2 handoff.")
            burger.deliver_full_stack(robot)

            print("[FSM] Delivered to Customer 2. Loading stop path 2.")
            load_pure_pursuit_path(robot, STOP_PATH_2)
            state = "DRIVING_TO_STOP"

        # -------------------------------------------------------------------
        # End Mission
        # -------------------------------------------------------------------

        elif state == "DRIVING_TO_STOP":
            nav_status = robot._nav_follow_pp_path_loop()

            if check_vision_class(robot, "stop sign"):
                print("[FSM] Stop sign detected. Halting platform and returning to IDLE.")
                robot.shutdown()
                state = "IDLE"

            elif nav_status != "MOVING":
                print("[FSM] Reached end of stop path without seeing stop sign. Halting.")
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

        if robot.get_button(Button.BTN_2):
            print("[FSM] BTN_2 pressed. Emergency stopping robot.")
            robot.shutdown()
            state = "IDLE"

        # Allow ROS callbacks to update vision match status.
        rclpy.spin_once(robot._node, timeout_sec=0.0)

        # -------------------------------------------------------------------
        # FSM Refresh Rate Control
        # -------------------------------------------------------------------

        next_tick += period
        sleep_s = next_tick - time.monotonic()

        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("robot")
    robot = Robot(node)

    try:
        run(robot)
    except KeyboardInterrupt:
        print("[FSM] Keyboard interrupt. Shutting down robot.")
        robot.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()