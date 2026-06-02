from __future__ import annotations


import time


from robot.hardware_map import Button, DEFAULT_FSM_HZ
from robot.robot import FirmwareState, Robot




# =========================
# STEPPER SETTINGS
# =========================


ELEVATION_STEPPER = 1


SHORTLIFT_HEIGHT_STEPS = 635


# picking up bottom bun
HEIGHT_1_STEPS = 1588
# lowering bottom bun
HEIGHT_2_STEPS = HEIGHT_1_STEPS + SHORTLIFT_HEIGHT_STEPS
# picking up patty
HEIGHT_3_STEPS = HEIGHT_1_STEPS
# lowering patty
HEIGHT_4_STEPS = HEIGHT_2_STEPS - 516
# picking up top bun
HEIGHT_5_STEPS = HEIGHT_1_STEPS - 397
# lowering bottom bun
HEIGHT_6_STEPS = HEIGHT_5_STEPS + SHORTLIFT_HEIGHT_STEPS
# going to origin
HEIGHT_7_STEPS = 516 + 397


STEPPER_MAX_VELOCITY = 500
STEPPER_ACCELERATION = 300
STEPPER_TIMEOUT = 10




# =========================
# SERVO SETTINGS
# =========================


GRIPPER_SERVO = 1
BELT_LEFT_SERVO = 2
BELT_RIGHT_SERVO = 3


GRIPPER_CLOSE_SPEED = 70
GRIPPER_OPEN_SPEED = 170


BELT_LEFT_IN_SPEED = 170
BELT_RIGHT_IN_SPEED = 70


BELT_LEFT_OUT_SPEED = 70
BELT_RIGHT_OUT_SPEED = 170


GRIPPER_TIME = 0.5
BELT_TIME = 3.0




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


    ok = robot.step_move(
        ELEVATION_STEPPER,
        steps,
        blocking=True,
        timeout=STEPPER_TIMEOUT,
    )


    print(f"[STEPPER] {label} done? {ok}")
    time.sleep(0.5)




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
    print("[TEST] Going down to Height 6")
    move_stepper(robot, HEIGHT_7_STEPS, "HEIGHT 7 DOWN")




def short_lift_up(robot: Robot) -> None:
    print("[TEST] Short Lift Up")
    move_stepper(robot, -SHORTLIFT_HEIGHT_STEPS, "SHORT LIFT UP")




# =========================
# SERVO FUNCTIONS
# =========================


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


def belts_outward(robot: Robot) -> None:
    print("[BELTS] Moving outward")


    robot.enable_servo(BELT_LEFT_SERVO)
    robot.enable_servo(BELT_RIGHT_SERVO)


    robot.set_servo(BELT_LEFT_SERVO, BELT_LEFT_OUT_SPEED)
    robot.set_servo(BELT_RIGHT_SERVO, BELT_RIGHT_OUT_SPEED)


    time.sleep(BELT_TIME)


    robot.disable_servo(BELT_LEFT_SERVO)
    robot.disable_servo(BELT_RIGHT_SERVO)




# =========================
# FULL BUTTON 1 SEQUENCE
# =========================


def full_stack_sequence(robot: Robot) -> None:
    print("[TEST] Starting Full Stack Sequence")


    # Bottom bun —-------------------
    go_to_height_1(robot)


    # Move rover to the shelf


    close_gripper(robot)
    short_lift_up(robot)


    # Move rover back
    
    belts_inward(robot)
    go_to_height_2(robot)
    open_gripper(robot)
    time.sleep(2)


    # Patty —-------------------
    go_to_height_3(robot)


    # Move rover to the shelf


    close_gripper(robot)
    short_lift_up(robot)


    # Move rover back


    belts_inward(robot)
    go_to_height_4(robot)
    open_gripper(robot)
    time.sleep(2)


    # Top bun —-------------------
    go_to_height_5(robot)


    # Move rover to the shelf


    close_gripper(robot)
    short_lift_up(robot)


    # Move rover back


    belts_inward(robot)
    go_to_height_6(robot)
    open_gripper(robot)
    # going to zero
    time.sleep(2)
    go_to_height_7(robot)


    print("[TEST] Full Stack Sequence Complete")

def deliver_full_stack(robot: Robot) -> None:
    print("[TEST] Starting Delivery Sequence")

    # Grip completed burger
    close_gripper(robot)
    time.sleep(1)

    # Lift burger
    go_to_height_1(robot)
    time.sleep(1)

    # Move rover to shelf

    # Deliver burger
    belts_outward(robot)

    # Release burger
    open_gripper(robot)

    print("[TEST] Delivery Sequence Complete")



# =========================
# MAIN LOOP
# =========================


def run(robot: Robot) -> None:
    print("[TEST] Burger Stack Test Ready")
    print("[TEST] BTN_1 = Run Full Stack Sequence")


    robot.set_state(FirmwareState.RUNNING)
    time.sleep(1)


    setup_elevation_stepper(robot)


    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()


    while True:
        if robot.was_button_pressed(Button.BTN_1):
            full_stack_sequence(robot)


        next_tick += period
        sleep_s = next_tick - time.monotonic()


        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()


