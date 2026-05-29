from __future__ import annotations

import time
import math
import rclpy
from std_srvs.srv import Trigger
from std_msgs.msg import Bool

import burger_assemblycode as burger
from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline
from robot.path_planner import PurePursuitPlanner

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

SCAN_PATH_CTRL   = [(0.0, 711.2), (0.0, 3352.8), (609.6, 3352.8), (609.6, 304.8), (1524.0, 304.8), (1524.0, 3352.8), (1828.8, 3352.8)]

CUST_1_PATH_CTRL = [(1828.8, 3352.8), (2133.6, 3352.8), (2133.6, 635.0)]
CUST_2_PATH_CTRL = [(2133.6, 635.0), (2133.6, 482.6)]

STOP_PATH_1 = [(2133.6, 635.0), (2133.6, 0.0)] 
STOP_PATH_2 = [(2133.6, 482.6), (2133.6, 0.0)] 

# Global Runtime Variables
current_vision_match = False
identified_customer = None

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

def load_pure_pursuit_path(robot: Robot, control_points: list) -> None:
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
# Helpers: Vision & Biometrics (ROS2 Clients)
# ---------------------------------------------------------------------------

def check_vision_class(robot: Robot, class_name: str, attribute_key: str | None = None, attribute_val: str | None = None) -> bool:
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return False

    for detection in robot.get_detections(class_name):
        if float(detection.get("confidence", 0)) < MIN_DETECTION_CONFIDENCE:
            continue
        if attribute_key and attribute_val:
            attributes = detection.get("attributes", {})
            if attributes.get(attribute_key, {}).get("value") != attribute_val:
                continue
        return True
    return False

def _vision_status_callback(msg):
    """Background listener updating the match status from face_tracker."""
    global current_vision_match
    current_vision_match = msg.data

def capture_and_encode_face(robot: Robot) -> bool:
    """Pings the background face_tracker node to lock in the target."""
    global identified_customer
    print("[VISION] Requesting classification from face_tracker...")
    
    # NOTE: Assuming your robot wrapper exposes the ROS node as robot.node
    client = robot.node.create_client(Trigger, '/vision/capture_target')
    
    if not client.wait_for_service(timeout_sec=2.0):
        print("[VISION] ERROR: face_tracker service is offline. Is it running?")
        return False
        
    req = Trigger.Request()
    future = client.call_async(req)
    
    # Briefly pause the FSM to wait for the camera to classify the face
    rclpy.spin_until_future_complete(robot.node, future)
    
    if future.result().success:
        identified_customer = future.result().message
        print(f"[VISION] Success! Customer identified as: {identified_customer}")
        
        # Subscribe to the live feed so we know when the target is matched later
        robot.node.create_subscription(Bool, '/vision/match_status', _vision_status_callback, 10)
        return True
        
    print(f"[VISION] FAILED: {future.result().message}")
    return False

def verify_live_face() -> bool:
    """Instantly returns the current math calculated by the background node."""
    global current_vision_match
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

        if state == "INIT":
            robot.set_state(FirmwareState.RUNNING)
            robot.reset_odometry()
            robot.wait_for_pose_update(timeout=0.2)
            print("[FSM] INIT Complete. Transitioning to IDLE.")
            state = "IDLE"

        # -- STEP 1: IDLE ---------------------------------------------------
        elif state == "IDLE":
            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 255)
            robot._draw_lidar_obstacles()
            
            if robot.get_button(Button.BTN_1):
                print("[FSM] BTN_1 pressed. Transitioning to WAITING_FOR_GREEN.")
                state = "WAITING_FOR_GREEN"

        # -- WAITING FOR GREEN LIGHT ----------------------------------------
        elif state == "WAITING_FOR_GREEN":
            if check_vision_class(robot, "traffic light", "color", "green"):
                print("[FSM] Green light! Starting Kitchen Sequence 1.")
                load_pure_pursuit_path(robot, KITCHEN_PATH_1)
                state = "NAV_KITCHEN_1"

        # ===================================================================
        # KITCHEN SEQUENCE 1 (BOTTOM BUN)
        # ===================================================================
        elif state == "NAV_KITCHEN_1":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Arrived at Item 1. Turning 90 degrees left.")
                state = "TURN_LEFT_ITEM_1"

        elif state == "TURN_LEFT_ITEM_1":
            robot.trigger_actuator_sequence("turn_left_90") 
            if robot.is_actuator_sequence_complete():
                state = "GRAB_ITEM_1"

        elif state == "GRAB_ITEM_1":
            burger.grab_and_lift_1(robot)
            print("[FSM] Item 1 secured. Turning 90 degrees right to face forward.")
            state = "TURN_RIGHT_ITEM_1"

        elif state == "TURN_RIGHT_ITEM_1":
            robot.trigger_actuator_sequence("turn_right_90")
            if robot.is_actuator_sequence_complete():
                state = "STOW_ITEM_1"

        elif state == "STOW_ITEM_1":
            burger.stow_item_1(robot)
            print("[FSM] Item 1 stowed. Loading Kitchen Path 2.")
            load_pure_pursuit_path(robot, KITCHEN_PATH_2)
            state = "NAV_KITCHEN_2"

        # ===================================================================
        # KITCHEN SEQUENCE 2 (PATTY)
        # ===================================================================
        elif state == "NAV_KITCHEN_2":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Arrived at Item 2. Turning 90 degrees left.")
                state = "TURN_LEFT_ITEM_2"

        elif state == "TURN_LEFT_ITEM_2":
            robot.trigger_actuator_sequence("turn_left_90") 
            if robot.is_actuator_sequence_complete():
                state = "GRAB_ITEM_2"

        elif state == "GRAB_ITEM_2":
            burger.grab_and_lift_2(robot)
            print("[FSM] Item 2 secured. Turning 90 degrees right to face forward.")
            state = "TURN_RIGHT_ITEM_2"

        elif state == "TURN_RIGHT_ITEM_2":
            robot.trigger_actuator_sequence("turn_right_90")
            if robot.is_actuator_sequence_complete():
                state = "STOW_ITEM_2"

        elif state == "STOW_ITEM_2":
            burger.stow_item_2(robot)
            print("[FSM] Item 2 stowed. Loading Kitchen Path 3.")
            load_pure_pursuit_path(robot, KITCHEN_PATH_3)
            state = "NAV_KITCHEN_3"

        # ===================================================================
        # KITCHEN SEQUENCE 3 (TOP BUN)
        # ===================================================================
        elif state == "NAV_KITCHEN_3":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Arrived at Item 3. Turning 90 degrees left.")
                state = "TURN_LEFT_ITEM_3"

        elif state == "TURN_LEFT_ITEM_3":
            robot.trigger_actuator_sequence("turn_left_90") 
            if robot.is_actuator_sequence_complete():
                state = "GRAB_ITEM_3"

        elif state == "GRAB_ITEM_3":
            burger.grab_and_lift_3(robot)
            print("[FSM] Item 3 secured. Turning 90 degrees right to face forward.")
            state = "TURN_RIGHT_ITEM_3"

        elif state == "TURN_RIGHT_ITEM_3":
            robot.trigger_actuator_sequence("turn_right_90")
            if robot.is_actuator_sequence_complete():
                state = "STOW_ITEM_3"

        elif state == "STOW_ITEM_3":
            burger.stow_item_3(robot)
            print("[FSM] Burger fully assembled! Navigating to Scan Station.")
            load_pure_pursuit_path(robot, SCAN_PATH_CTRL)
            state = "NAV_TO_CUSTOMER_SCAN"

        # ===================================================================
        # DELIVERY & VERIFICATION
        # ===================================================================
        elif state == "NAV_TO_CUSTOMER_SCAN":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Scan station reached. Attempting biometric capture...")
                state = "MEMORIZING_TARGET"

        elif state == "MEMORIZING_TARGET":
            robot.set_led(LED.BLUE, 255) 
            if capture_and_encode_face(robot):
                print(f"[FSM] Profile locked ({identified_customer}). Loading Drop-off Path.")
                robot.set_led(LED.BLUE, 0)
                load_pure_pursuit_path(robot, CUST_1_PATH_CTRL)
                state = "NAV_TO_DROPOFF"
            else:
                time.sleep(0.5) 

        elif state == "NAV_TO_DROPOFF":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Reached Drop-off zone. Turning left to Customer 1.")
                state = "TURN_TO_CUST_1"

        elif state == "TURN_TO_CUST_1":
            robot.trigger_actuator_sequence("turn_left_90")
            if robot.is_actuator_sequence_complete():
                print("[FSM] Facing Customer 1. Verifying biometric matrix...")
                state = "VERIFY_CUST_1"

        elif state == "VERIFY_CUST_1":
            if verify_live_face(): # Checking the background node status instantly
                print("[VISION] MATCH! Target is Customer 1.")
                robot.set_led(LED.GREEN, 255)
                state = "DELIVERING_CUST_1" 
            else:
                print("[VISION] NO MATCH. Re-aligning...")
                robot.set_led(LED.RED, 255)
                state = "REJECT_CUST_1_TURN_BACK"

        elif state == "REJECT_CUST_1_TURN_BACK":
            robot.trigger_actuator_sequence("turn_right_90")
            if robot.is_actuator_sequence_complete():
                print("[FSM] Re-aligned. Loading path to Customer 2.")
                load_pure_pursuit_path(robot, CUST_2_PATH_CTRL) 
                state = "NAV_TO_CUST_2"

        elif state == "NAV_TO_CUST_2":
            if robot._nav_follow_pp_path_loop() != "MOVING":
                print("[FSM] Reached Customer 2. Verifying...")
                state = "VERIFY_CUST_2"
        
        elif state == "VERIFY_CUST_2":
            if verify_live_face():
                print("[VISION] MATCH! Target is Customer 2.")
                robot.set_led(LED.GREEN, 255)
                state = "DELIVERING_CUST_2" 
            else:
                print("[FSM] FATAL ERROR: Neither customer matched.")
                robot.shutdown()
                state = "IDLE"

        # ===================================================================
        # PAYLOAD HANDOFF & END MISSION
        # ===================================================================
        elif state == "DELIVERING_CUST_1":
            print("[FSM] Initiating Customer 1 Hand-off...")
            burger.dropoff_burger(robot)
            
            print("[FSM] Delivered to Cust 1. Loading Stop Path 1.")
            load_pure_pursuit_path(robot, STOP_PATH_1)
            state = "DRIVING_TO_STOP"

        elif state == "DELIVERING_CUST_2":
            print("[FSM] Initiating Customer 2 Hand-off...")
            burger.dropoff_burger(robot)
            
            print("[FSM] Delivered to Cust 2. Loading Stop Path 2.")
            load_pure_pursuit_path(robot, STOP_PATH_2)
            state = "DRIVING_TO_STOP"
            
        elif state == "DRIVING_TO_STOP":
            nav_status = robot._nav_follow_pp_path_loop()
            
            if check_vision_class(robot, "stop sign"):
                print("[FSM] Stop sign detected! Halting platform and returning to IDLE.")
                robot.shutdown()
                state = "IDLE"
            elif nav_status != "MOVING":
                print("[FSM] Reached end of path without seeing stop sign. Halting.")
                state = "IDLE"

        # -- Emergency Stop Interrupt
        if robot.get_button(Button.BTN_2):
            print("BTN_2 pressed. Emergency stopping robot.")
            robot.shutdown()
            state = "IDLE"

        # -- FSM Refresh Rate Control --------------------------------------
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()