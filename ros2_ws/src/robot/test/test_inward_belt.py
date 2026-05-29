from __future__ import annotations

import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ
from robot.robot import FirmwareState, Robot


BELT_LEFT_SERVO = 3
BELT_RIGHT_SERVO = 2

BELT_LEFT_IN_SPEED = 70
BELT_RIGHT_IN_SPEED = 110

BELT_TIME = 3.0


def belts_inward(robot: Robot) -> None:
    print("[BELTS] Moving inward")
    robot.enable_servo(BELT_LEFT_SERVO)
    robot.enable_servo(BELT_RIGHT_SERVO)

    robot.set_servo(BELT_LEFT_SERVO, BELT_LEFT_IN_SPEED)
    robot.set_servo(BELT_RIGHT_SERVO, BELT_RIGHT_IN_SPEED)

    time.sleep(BELT_TIME)

    robot.disable_servo(BELT_LEFT_SERVO)
    robot.disable_servo(BELT_RIGHT_SERVO)


def run(robot: Robot) -> None:
    print("[TEST] Press BTN_5 to run belts inward")
    robot.set_state(FirmwareState.RUNNING)

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if robot.was_button_pressed(Button.BTN_5):
            belts_inward(robot)

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()