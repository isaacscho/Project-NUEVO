from __future__ import annotations

import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ
from robot.robot import FirmwareState, Robot


# =========================
# STEPPER SETTINGS
# =========================

ELEVATION_STEPPER = 1

SHORTLIFT_HEIGHT_STEPS = 635

# Heights
HEIGHT_1_STEPS = 1111
HEIGHT_2_STEPS = 1230
HEIGHT_3_STEPS = 833
HEIGHT_4_STEPS = 1746
HEIGHT_5_STEPS = 595
HEIGHT_6_STEPS = 913

STEPPER_MAX_VELOCITY = 500
STEPPER_ACCELERATION = 300
STEPPER_TIMEOUT = 10


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


def move_stepper(robot: Robot, steps: int, label: str) -> None:
    print(f"[STEPPER] {label}: {steps} steps")

    robot.step_move(
        ELEVATION_STEPPER,
        steps,
        blocking=True,
        timeout=STEPPER_TIMEOUT,
    )

    print("[STEPPER] Move complete")


# =========================
# PREDEFINED FUNCTIONS
# =========================

# Bottom bun up
def grab_bottom_bun(robot: Robot) -> None:
    print("[TEST] Bottom Bun Up")

    move_stepper(robot, -HEIGHT_1_STEPS, "HEIGHT_1 UP")


# Bottom bun down
def lower_bottom_bun(robot: Robot) -> None:
    print("[TEST] Bottom Bun Down")

    move_stepper(robot, HEIGHT_4_STEPS, "HEIGHT_4 DOWN")


# Patty up
def grab_patty(robot: Robot) -> None:
    print("[TEST] Patty Up")

    move_stepper(robot, -HEIGHT_1_STEPS, "HEIGHT_1 UP AGAIN")


# Patty down
def lower_patty(robot: Robot) -> None:
    print("[TEST] Patty Down")

    move_stepper(robot, HEIGHT_2_STEPS, "HEIGHT_2 DOWN")


# Top bun up
def grab_top_bun(robot: Robot) -> None:
    print("[TEST] Top Bun Up")

    move_stepper(robot, -HEIGHT_5_STEPS, "HEIGHT_5 UP")


# Top bun down
def lower_top_bun(robot: Robot) -> None:
    print("[TEST] Top Bun Down")

    move_stepper(robot, HEIGHT_3_STEPS, "HEIGHT_3 DOWN")


# Return to origin
def return_home(robot: Robot) -> None:
    print("[TEST] Return Home")

    move_stepper(robot, HEIGHT_6_STEPS, "HEIGHT_6 DOWN")


# Short lift up
def short_lift_up(robot: Robot) -> None:
    print("[TEST] Short Lift Up")

    move_stepper(robot, -SHORTLIFT_HEIGHT_STEPS, "SHORT LIFT UP")


# Short lift down
def short_lift_down(robot: Robot) -> None:
    print("[TEST] Short Lift Down")

    move_stepper(robot, SHORTLIFT_HEIGHT_STEPS, "SHORT LIFT DOWN")


def stop_stepper(robot: Robot) -> None:
    print("[STEPPER] Disabling stepper")

    robot.step_disable(ELEVATION_STEPPER)


# =========================
# MAIN LOOP
# =========================

def run(robot: Robot) -> None:
    print("[TEST] Stepper Function Test Ready")

    print("[TEST] BTN_1 = Bottom Bun Up")
    print("[TEST] BTN_2 = Bottom Bun Down")
    print("[TEST] BTN_3 = Patty Up")
    print("[TEST] BTN_4 = Patty Down")
    print("[TEST] BTN_5 = Top Bun Up")
    print("[TEST] BTN_6 = Top Bun Down")

    robot.set_state(FirmwareState.RUNNING)

    setup_elevation_stepper(robot)

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        # Bottom bun up
        if robot.was_button_pressed(Button.BTN_1):
            grab_bottom_bun(robot)

        # Bottom bun down
        if robot.was_button_pressed(Button.BTN_2):
            lower_bottom_bun(robot)

        # Patty up
        if robot.was_button_pressed(Button.BTN_3):
            grab_patty(robot)

        # Patty down
        if robot.was_button_pressed(Button.BTN_4):
            lower_patty(robot)

        # Top bun up
        if robot.was_button_pressed(Button.BTN_5):
            grab_top_bun(robot)

        # Top bun down
        if robot.was_button_pressed(Button.BTN_6):
            lower_top_bun(robot)

        next_tick += period
        sleep_s = next_tick - time.monotonic()

        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()