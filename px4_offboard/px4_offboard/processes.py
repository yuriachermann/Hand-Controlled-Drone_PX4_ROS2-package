#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Run the PX4 SITL simulation
    # "cd ~/PX4-Autopilot && make px4_sitl gz_x500",
    "cd ~/PX4-Autopilot && PX4_GZ_MODEL_NAME=x500 make px4_sitl gz_x500",

    # Visualize Leap Motion Hand Tracking
    "python3 ~/ros2_px4_offboard_example_ws/src/ROS2_PX4_Offboard_Example/px4_offboard/px4_offboard/hand_visualizer.py",

    # Run QGroundControl
    # "cd ~/QGroundControl && ./QGroundControl.AppImage",
]

# Loop through each command in the list
for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(1)
