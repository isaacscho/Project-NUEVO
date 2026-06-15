# Project NUEVO: Autonomous Burger Assembly & Delivery

![Project NUEVO Thumbnail](assets/IMG_3436.png)

## Project Overview
Project-NUEVO is a modular two-wheeled mobile robot engineered to autonomously assemble and deliver burgers in a tight kitchen environment. Built on a dual-layer control architecture, the system navigates a complex course, recognizes traffic signals, performs multi-stage mechanical manipulation at specific kitchen stations, avoids physical obstacles using dynamic path planning, and uses biometric facial recognition to deliver the final product to the correct customer.

## Problem
Automating kitchen assembly requires high precision in incredibly constrained spaces. Standard front-wheel drive and head-on manipulation designs proved prone to collision and required too much space to maneuver. Furthermore, the robot required a robust software architecture capable of handling heavy computer vision tasks (like YOLO and facial recognition) simultaneously with real-time motor control and obstacle avoidance, all without overwhelming the onboard Raspberry Pi 5.

## Design And Approach

### Mechanical
* **Drive Configuration:** Converted the chassis from Front-Wheel Drive (FWD) to Rear-Wheel Drive (RWD) to improve turning dynamics, stability, and spatial efficiency.
* **Drive-By Manipulation:** Designed a left-mounted, raised sideways claw. Instead of driving head-on into the counter, the robot shifts into a parallel drive lane (X = -180.0) and uses a custom stepper-driven elevator to reach *over* the counter, eliminating chassis collision risks.

### Electrical
* **Actuation:** Driven by DC motors for the main chassis, paired with a precision stepper motor for the elevation lift and servos for the gripper.
* **Custom PCB:** Integrates the Arduino, motor drivers, and power management into a standardized interface for maximum reproducibility.
* **Sensors:** Integrated front-focused LiDAR (filtered to a 110-degree FOV) for spatial mapping and a loopback camera device for visual processing.

### Software
* **Master FSM (ROS 2):** Engineered a highly threaded Finite State Machine handling distinct sequences for bottom buns, patties, top buns, and a two-stage delivery handoff to prevent payload drops.
* **Navigation:** Implemented Leashed Artificial Potential Fields (LAPF) to project a virtual "leash" that safely guides the RWD geometry through tight obstacle corridors, bypassing the physical limitations of standard pure pursuit algorithms.
* **Vision Pipeline:** Centralized all visual processing into a single ROS 2 `vision_node`. It utilizes YOLO (NCNN) for real-time traffic light and stop sign detection, and a `face_recognition` service (dlib) that runs on-demand to identify biometric targets without bottlenecking CPU performance during motion.
* **Low-Level Control:** Real-time embedded C++ on Arduino handling UART communication, GPIO, and precise motor state execution.

## Results
* **Navigation:** The switch to LAPF completely resolved previous "blind nose" collision bugs associated with the RWD geometry, successfully maneuvering the robot through the LiDAR obstacle section.
* **Vision & State Management:** Successfully decoupled heavy `dlib` facial encoding from the main control loop by placing it behind a triggered ROS 2 service, maintaining a stable, high FSM tick rate and fast YOLO inference times during active driving.
* **Manipulation:** The two-stage delivery sequence (prepping the burger at a shared waypoint before executing the final shelf approach) significantly increased delivery success rates by preventing mid-turn payload drops.

## How To Reproduce

### 1. Build the Workspace
```bash
git clone [https://github.com/isaacscho/Project-NUEVO](https://github.com/isaacscho/Project-NUEVO)
cd Project-NUEVO/ros2_ws
colcon build
source install/setup.bash
