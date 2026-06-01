from __future__ import annotations

import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ
from robot.robot import FirmwareState, Robot


GRIPPER_SERVO = 1
GRIPPER_OPEN_SPEED = 110
GRIPPER_TIME = 0.5


def open_gripper(robot: Robot) -> None:
    print("[GRIPPER] Opening")
    robot.enable_servo(GRIPPER_SERVO)
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_SPEED)
    time.sleep(GRIPPER_TIME)
    robot.disable_servo(GRIPPER_SERVO)


def run(robot: Robot) -> None:
    print("[TEST] Press BTN_5 to open gripper")
    robot.set_state(FirmwareState.RUNNING)

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if robot.was_button_pressed(Button.BTN_5):
            open_gripper(robot)

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()