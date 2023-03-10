#!/bin/bash
gnome-terminal --tab --title="launch_ocean_world.launch" -- bash -c "roslaunch auto_landing launch_ocean_world.launch"

sleep 5

# Open a new terminal tab and execute landpad_det
gnome-terminal --tab --title="landpad_det" -- bash -c "rosrun auto_landing landpad_det"

# Open a new terminal tab and execute detect_with_front_camera.py
gnome-terminal --tab --title="detect_with_front_camera.py" -- bash -c "rosrun auto_landing detect_with_front_camera.py"

# Open a new terminal tab and launch launch_ocean_mission.launch
gnome-terminal --tab --title="ocean_mission" -- bash -c "roslaunch auto_landing launch_ocean_mission.launch"

cd scripts/analysis
gnome-terminal --tab --title="record_link.py " -- bash -c "python record_link.py"

# Open a new terminal tab and launch launch_ocean_mission.launch
gnome-terminal --tab --title="px4_controller_withcamera.py " -- bash -c "rosrun auto_landing px4_controller_withcamera.py "


# Exit the script
exit 0
