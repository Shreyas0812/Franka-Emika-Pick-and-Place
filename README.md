# Franka Emika Pick-and-Place — MEAM 5200 Final Project

**University of Pennsylvania · MEAM 5200 (Introduction to Robotics) · Fall 2024 · Team 17**

Autonomous pick-and-place and block-stacking with a 7-DOF **Franka Emika Panda** arm. The robot detects, grasps, transports, and stacks blocks from both a **static** table and a **dynamic** rotating turntable, first validated in simulation (Gazebo/RViz) and then deployed on real hardware in a timed red-vs-blue competition.

<p align="center">
  <img src="images/competition_stack.jpg" alt="Competition stack" width="480">
</p>

## Overview

Each team has a fixed time window to stack as many blocks as high as possible. Points scale with stack height, so the strategy prioritizes reliable, well-aligned placement over raw speed. The solution combines:

- **Static pick-and-place** — detect blocks on the table via the arm camera, correct their orientation, and stack them in a target zone.
- **Dynamic pick-and-place** — intercept blocks moving on a rotating turntable using a complementary filter to smooth noisy detections, then stack them on top of the static tower.

## Approach

| Component | Description |
|-----------|-------------|
| Forward Kinematics | DH-based FK for end-effector pose and Jacobian (`lib/calculateFK.py`, `lib/calculateFKJac.py`) |
| Inverse Kinematics | Damped least-squares / pseudo-inverse IK with null-space optimization (`lib/IK_position_null.py`) |
| Velocity Kinematics | Jacobian-based velocity IK and manipulability (`lib/IK_velocity*.py`, `lib/calcManipulability.py`) |
| Motion Planning | RRT and potential-field planners for collision-free paths (`lib/rrt.py`, `lib/potentialFieldPlanner.py`, `lib/detectCollision.py`) |
| Perception | AprilTag-based block detection with a complementary filter for the moving turntable |
| Robustness | Pre-computed "emergency" joint configurations used as fallbacks if an IK solve fails mid-run |

## Repository Structure

```
final.py              Main competition script (static + dynamic stacking)
finalRRT.py           Variant using RRT-based motion planning
final_static.py       Static-only pick-and-place
lib/                  Kinematics, planning, and perception library
hardware_tested/      Versions validated on the physical robot
old_files/            Development history / earlier iterations
SUBMIT/               Final graded submission (Team17.zip)
images/               Result photos and demo videos
MEAM5200_Final_Project_Report.pdf            Team report
MEAM5200_Final_Project___Individual_Report.pdf   Individual report
MEAM5200_FinalProject.pptx                   Presentation
```

## Running

This project runs on top of the MEAM 5200 course ROS stack (`core.interfaces`, `core.utils`) and is not standalone. Within the course environment:

```bash
# Launch the simulation or hardware environment (sets the 'team' param to red/blue)
roslaunch <course_package> final.launch

# In a second terminal, run the competition script
python final.py
```

The script moves to a start configuration, waits for ENTER, then executes the static stack followed by the dynamic stack, printing timing for each phase.

## Results

The system successfully stacked blocks from both the static table and the dynamic turntable in simulation and on hardware. See `images/` for demo videos (`full_stack_video.mp4`, `simulation_dynamic_stack.mp4`) and result photos.

## Dependencies

Python 3.8, NumPy, ROS (course-provid