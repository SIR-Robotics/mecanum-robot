# Mecanum Robot Agent Guide

This repo is the firmware for the mobile mecanum robot in the NSEP AI Robotics system.

The robot fetches blocks and returns them to the docking station. It drives by following black lines in a grid-based map, detects the block/color state, runs the matching pickup action, then returns.

## Current Focus

We are working on Challenge 3.

Challenge 3 has three pickup routes:

- `path1`
- `path2`
- `path3`

Keep route changes focused on those paths unless the task explicitly says otherwise.

## Map Context

The environment is a line grid. Use this image as the map reference:

![Challenge grid](context/grid.png)

Think of each cell edge/intersection as part of the robot's line-following route. The robot should move along grid lines, count intersections/turns as needed, fetch the block, and return to docking.

## Firmware Notes

- PlatformIO project.
- Arduino Uno build uses `src/main.cpp`.
- ESP32/Favoriot bridge build uses `src/esp32_wifi.cpp`.
- Check `platformio.ini` before changing build targets.
- Keep hardware tuning constants visible; real motors, sensors, and turns need calibration.

## Editing Rules

- Prefer small route/state-machine changes over new abstractions.
- Reuse existing movement, line-following, color, gripper, and ESP32 command helpers.
- Do not add a new dependency unless the existing firmware cannot reasonably do the job.
- For path logic, add the smallest runnable check or serial-observable behavior that proves the route still works.
