from __future__ import annotations

import time

from robot.robot import Robot


# =========================
# STEPPER SETTINGS
# =========================

ELEVATION_STEPPER = 1

SHORTLIFT_HEIGHT_STEPS = 350
CLEARANCE_HEIGHT = 525
CLEARANCE_HEIGHT_DOWN = 525

# picking up bottom bun
HEIGHT_1_STEPS = 1463

# lowering bottom bun / assembled burger pickup height
HEIGHT_2_STEPS = HEIGHT_1_STEPS + CLEARANCE_HEIGHT

# picking up patty
HEIGHT_3_STEPS = HEIGHT_1_STEPS

# lowering patty
HEIGHT_4_STEPS = HEIGHT_2_STEPS - 516

# picking up top bun
HEIGHT_5_STEPS = HEIGHT_1_STEPS - 397

# lowering top bun
HEIGHT_6_STEPS = HEIGHT_5_STEPS + SHORTLIFT_HEIGHT_STEPS

# returning toward origin
HEIGHT_7_STEPS = 516 + 397

STEPPER_MAX_VELOCITY = 500
STEPPER_ACCELERATION = 300
STEPPER_TIMEOUT = 10


# =========================
# SERVO SETTINGS
# =========================

GRIPPER_SERVO = 1
BELT_LEFT_SERVO = 2
BELT_RIGHT_SERVO = 4

GRIPPER_CLOSE_SPEED = 70
GRIPPER_OPEN_SPEED = 170

BELT_LEFT_IN_SPEED = 170
BELT_RIGHT_IN_SPEED = 70

BELT_LEFT_OUT_SPEED = 70
BELT_RIGHT_OUT_SPEED = 170

GRIPPER_TIME_Bun = 0.9
GRIPPER_TIME_Patty = 0.95
BELT_TIME = 6.0
BELT_TIME_Deliv = 4

# =========================
# SETUP
# =========================

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


def move_stepper(robot: Robot, steps: int, label: str) -> bool:
    print(f"[STEPPER] {label}: {steps} steps")

    ok = robot.step_move(
        ELEVATION_STEPPER,
        steps,
        blocking=True,
        timeout=STEPPER_TIMEOUT,
    )

    print(f"[STEPPER] {label} done? {ok}")
    time.sleep(0.5)
    return bool(ok)


# =========================
# STEPPER FUNCTIONS
# =========================

def go_to_height_1(robot: Robot) -> None:
    print("[TEST] Going to Height 1")
    move_stepper(robot, -HEIGHT_1_STEPS, "HEIGHT 1 UP")


def go_to_height_2(robot: Robot) -> None:
    print("[TEST] Going down to Height 2")
    move_stepper(robot, HEIGHT_2_STEPS, "HEIGHT 2 DOWN")


def go_to_height_3(robot: Robot) -> None:
    print("[TEST] Going up to Height 3 for patty")
    move_stepper(robot, -HEIGHT_3_STEPS, "HEIGHT 3 UP")


def go_to_height_4(robot: Robot) -> None:
    print("[TEST] Going down to Height 4")
    move_stepper(robot, HEIGHT_4_STEPS, "HEIGHT 4 DOWN")


def go_to_height_5(robot: Robot) -> None:
    print("[TEST] Going up to Height 5 for top bun")
    move_stepper(robot, -HEIGHT_5_STEPS, "HEIGHT 5 UP")


def go_to_height_6(robot: Robot) -> None:
    print("[TEST] Going down to Height 6")
    move_stepper(robot, HEIGHT_6_STEPS, "HEIGHT 6 DOWN")


def go_to_height_7(robot: Robot) -> None:
    print("[TEST] Going down to Height 7")
    move_stepper(robot, HEIGHT_7_STEPS, "HEIGHT 7 DOWN")


def short_lift_up(robot: Robot) -> None:
    print("[TEST] Short Lift Up")
    move_stepper(robot, -SHORTLIFT_HEIGHT_STEPS, "SHORT LIFT UP")


def short_lift_down(robot: Robot) -> None:
    print("[TEST] Short Lift Down")
    move_stepper(robot, SHORTLIFT_HEIGHT_STEPS, "SHORT LIFT DOWN")


def clearance_height_up(robot: Robot) -> None:
    print("[TEST] CLEAR HEIGHT")
    move_stepper(robot, -CLEARANCE_HEIGHT, "CLEAR HEIGHT")


def clearance_height_down(robot: Robot) -> None:
    print("[TEST] CLEAR HEIGHT DOWN")
    move_stepper(robot, CLEARANCE_HEIGHT_DOWN, "CLEAR HEIGHT DOWN")


# =========================
# SERVO FUNCTIONS
# =========================

def close_gripper_Bun(robot: Robot) -> None:
    print("[GRIPPER] Closing")

    robot.enable_servo(GRIPPER_SERVO)
    robot.set_servo(GRIPPER_SERVO, GRIPPER_CLOSE_SPEED)
    time.sleep(GRIPPER_TIME_Bun)
    robot.disable_servo(GRIPPER_SERVO)

def close_gripper_Patty(robot: Robot) -> None:
    print("[GRIPPER] Closing")

    robot.enable_servo(GRIPPER_SERVO)
    robot.set_servo(GRIPPER_SERVO, GRIPPER_CLOSE_SPEED)
    time.sleep(GRIPPER_TIME_Patty)
    robot.disable_servo(GRIPPER_SERVO)

def open_gripper_Bun(robot: Robot) -> None:
    print("[GRIPPER] Opening")

    robot.enable_servo(GRIPPER_SERVO)
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_SPEED)
    time.sleep(GRIPPER_TIME_Bun)
    robot.disable_servo(GRIPPER_SERVO)

def open_gripper_Patty(robot: Robot) -> None:
    print("[GRIPPER] Opening")

    robot.enable_servo(GRIPPER_SERVO)
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_SPEED)
    time.sleep(GRIPPER_TIME_Patty)
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


def belts_outward(robot: Robot) -> None:
    print("[BELTS] Moving outward")

    robot.enable_servo(BELT_LEFT_SERVO)
    robot.enable_servo(BELT_RIGHT_SERVO)

    robot.set_servo(BELT_LEFT_SERVO, BELT_LEFT_OUT_SPEED)
    robot.set_servo(BELT_RIGHT_SERVO, BELT_RIGHT_OUT_SPEED)

    time.sleep(BELT_TIME_Deliv)

    robot.disable_servo(BELT_LEFT_SERVO)
    robot.disable_servo(BELT_RIGHT_SERVO)


def stop_belts(robot: Robot) -> None:
    print("[BELTS] Stopping")

    robot.disable_servo(BELT_LEFT_SERVO)
    robot.disable_servo(BELT_RIGHT_SERVO)


# =========================
# BURGER PICKUP SEQUENCES
# =========================

def BottomBun_sequence(robot: Robot) -> None:
    print("[TEST] Starting Bottom Bun Sequence")

    clearance_height_down(robot)
    close_gripper_Bun(robot)
    short_lift_up(robot)

    belts_inward(robot)
    time.sleep(2)

    open_gripper_Bun(robot)
    time.sleep(2)

    short_lift_down(robot)
    clearance_height_up(robot)

    print("[TEST] Bottom Bun Sequence Complete")


def Patty_sequence(robot: Robot) -> None:
    print("[TEST] Starting Patty Sequence")

    clearance_height_down(robot)
    close_gripper_Patty(robot)
    short_lift_up(robot)

    belts_inward(robot)
    time.sleep(2)

    open_gripper_Patty(robot)
    time.sleep(2)

    short_lift_down(robot)
    clearance_height_up(robot)

    print("[TEST] Patty Sequence Complete")


def TopBun_sequence(robot: Robot) -> None:
    print("[TEST] Starting Top Bun Sequence")

    clearance_height_down(robot)
    close_gripper_Bun(robot)
    short_lift_up(robot)

    belts_inward(robot)
    time.sleep(2)

    open_gripper_Bun(robot)
    time.sleep(2)

    short_lift_down(robot)
    clearance_height_up(robot)

    print("[TEST] Top Bun Sequence Complete")


# =========================
# DELIVERY SEQUENCES
# =========================

def deliver_full_stack(robot: Robot) -> None:
    """
    Pick up the completed burger stack before driving to customer shelf.
    Call this before leaving the burger assembly area if the robot needs
    to grip and lift the completed burger.
    """
    print("[TEST] Starting Delivery Pickup Sequence")

    # Lower to completed burger pickup height
    go_to_height_2(robot)

    # Grip completed burger
    close_gripper_Bun(robot)

    # Lift burger for driving
    go_to_height_1(robot)
    time.sleep(1)

    print("[TEST] Delivery Pickup Sequence Complete")


def deliver_burger_final(robot: Robot) -> None:
    """
    Final drop-off at customer shelf.
    Call this after the robot reaches the correct delivery location.
    """
    print("[TEST] Starting Final Burger Dropoff")

    belts_outward(robot)
    time.sleep(1)

    open_gripper_Bun(robot)

    print("[TEST] Final Burger Dropoff Complete")


# =========================
# OPTIONAL FULL TEST SEQUENCE
# =========================

def full_burger_test_sequence(robot: Robot) -> None:
    """
    Optional bench/full-system test.
    Do not call this from the main FSM unless you intentionally want
    to run all burger actions in one sequence.
    """
    print("[TEST] Starting Full Burger Test Sequence")

    setup_elevation_stepper(robot)

    go_to_height_1(robot)
    clearance_height_up(robot)

    BottomBun_sequence(robot)
    Patty_sequence(robot)
    TopBun_sequence(robot)

    deliver_full_stack(robot)
    deliver_burger_final(robot)

    print("[TEST] Full Burger Test Sequence Complete")