from __future__ import annotations
import time
from robot.robot import Robot

# =========================
# STEPPER SETTINGS
# =========================

ELEVATION_STEPPER = 1
SHORTLIFT_HEIGHT_STEPS = 635

# Height going up for bottom bun and patty
HEIGHT_1_STEPS = 1111.25
# Height going down with bottom bun
HEIGHT_4_STEPS = HEIGHT_1_STEPS + SHORTLIFT_HEIGHT_STEPS
# Height going down with patty 
HEIGHT_2_STEPS = HEIGHT_4_STEPS - 515.9375
# Height going down with top bun
HEIGHT_3_STEPS = HEIGHT_4_STEPS - 515.9375 - 396.875

# --- NEW FIXED VARIABLES ---
# Placeholder: Adjust this to the actual top bun grab height
HEIGHT_5_STEPS = HEIGHT_1_STEPS + 500 
# Placeholder: Assuming origin is 0 steps
HEIGHT_6_STEPS = 0 
# ---------------------------

STEPPER_MAX_VELOCITY = 500
STEPPER_ACCELERATION = 300
STEPPER_TIMEOUT = 30


def setup_elevation_stepper(robot: Robot) -> None:
    print("[STEPPER] Enabling stepper")
    robot.step_enable(ELEVATION_STEPPER)
    time.sleep(0.5)
    robot.step_set_config(
        ELEVATION_STEPPER,
        max_velocity=STEPPER_MAX_VELOCITY,
        acceleration=STEPPER_ACCELERATION,
    )
    time.sleep(0.5)

def move_elevator(robot: Robot, steps: int, label: str) -> None:
    print(f"[STEPPER] {label}: {steps} steps")
    ok = robot.step_move(
        ELEVATION_STEPPER,
        steps,
        blocking=True,
        timeout=STEPPER_TIMEOUT,
    )
    print(f"[STEPPER] {label} done? {ok}")
    time.sleep(0.5)


# =========================
# STEPPER MOTOR FUNCTIONS
# =========================

def grab_bottom_bun(robot: Robot) -> None:
    move_elevator(robot, -HEIGHT_1_STEPS, "Going up to bottom bun")

def lower_bottom_bun(robot: Robot) -> None:
    move_elevator(robot, HEIGHT_4_STEPS, "Going down with bottom bun")

def grab_patty(robot: Robot) -> None:
    move_elevator(robot, -HEIGHT_1_STEPS, "Going up to patty")

def lower_patty(robot: Robot) -> None:
    move_elevator(robot, HEIGHT_2_STEPS, "Going down with patty")

def grab_top_bun(robot: Robot) -> None:
    move_elevator(robot, -HEIGHT_5_STEPS, "Going up to top bun")

def lower_top_bun(robot: Robot) -> None:
    move_elevator(robot, HEIGHT_3_STEPS, "Going down with top bun")

def short_lift_up(robot: Robot) -> None:
    move_elevator(robot, -SHORTLIFT_HEIGHT_STEPS, "Short lift up")

def return_to_origin(robot: Robot) -> None:
    move_elevator(robot, HEIGHT_6_STEPS, "Returning to origin")


# =========================
# SERVO SETTINGS
# =========================

GRIPPER_SERVO = 1
BELT_LEFT_SERVO = 3
BELT_RIGHT_SERVO = 2

GRIPPER_CLOSE_SPEED = 70
GRIPPER_OPEN_SPEED = 110

BELT_LEFT_IN_SPEED = 70
BELT_RIGHT_IN_SPEED = 110

GRIPPER_TIME = 0.5
BELT_TIME = 3.0


def close_gripper(robot: Robot) -> None:
    print("[GRIPPER] Closing")
    robot.enable_servo(GRIPPER_SERVO)
    robot.set_servo(GRIPPER_SERVO, GRIPPER_CLOSE_SPEED)
    time.sleep(GRIPPER_TIME)
    robot.disable_servo(GRIPPER_SERVO)

def open_gripper(robot: Robot) -> None:
    print("[GRIPPER] Opening")
    robot.enable_servo(GRIPPER_SERVO)
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_SPEED)
    time.sleep(GRIPPER_TIME)
    robot.disable_servo(GRIPPER_SERVO)

def belts_inward(robot: Robot) -> None:
    print("[BELTS] Moving inward")
    robot.enable_servo(BELT_LEFT_SERVO)
    robot.enable_servo(BELT_RIGHT_SERVO)
    robot.set_servo(BELT_LEFT_SERVO, BELT_LEFT_IN_SPEED)
    robot.set_servo(BELT_RIGHT_SERVO, BELT_RIGHT_IN_SPEED)
    time.sleep(BELT_TIME)
    robot.disable_servo(BELT_LEFT_SERVO)
    robot.disable_servo(BELT_RIGHT_SERVO)


# =========================
# EXPORTED WRAPPER FUNCTIONS
# (Called by main.py)
# =========================

def grab_and_lift_1(robot: Robot) -> None:
    setup_elevation_stepper(robot)
    grab_bottom_bun(robot)
    close_gripper(robot)
    short_lift_up(robot)

def stow_item_1(robot: Robot) -> None:
    belts_inward(robot)
    lower_bottom_bun(robot)
    open_gripper(robot)

def grab_and_lift_2(robot: Robot) -> None:
    grab_patty(robot)
    close_gripper(robot)
    short_lift_up(robot)

def stow_item_2(robot: Robot) -> None:
    belts_inward(robot)
    lower_patty(robot)
    open_gripper(robot)

def grab_and_lift_3(robot: Robot) -> None:
    grab_top_bun(robot)
    close_gripper(robot)
    short_lift_up(robot)

def stow_item_3(robot: Robot) -> None:
    belts_inward(robot)
    lower_top_bun(robot)
    open_gripper(robot)
    return_to_origin(robot) # Reset elevator when finished building