#!/bin/bash
set -e

# Run the robot simulation and save the animation
python /app/src/robot.py

# Play the saved animation (requires X11 GUI setup)
ffplay -autoexit -fs /app/rrr_animation.mp4
