# Cylinder Estimation and Marker Landing
This mission integrates two essential tasks for autonomous drone operations: cylinder estimation through circular flight and an Image-Based Visual Servoing landing utilizing ArUco-marked cylinders.

## Starting the Mission
### Step 1: Load the Simulation Environment
Run the following command to load the simulation:
ros2 launch mission cylinder_landing.launch.py

### Step 2: Run the ArUco Marker Detection
To detect ArUco markers, execute:
ros2 run mission aruco_tracker.py

### Step 3: Execute the Mission
Start the mission by running:
ros2 run mission MissionFile.py

## Mission Phases
### Stage 1: Cylinder Estimation (Autonomous Circular Flight)
Auto-arm and OFFBOARD Mode: The drone enters OFFBOARD mode, enabling ROS-based control.
Takeoff to (0, 0, -5): The drone ascends to a height of -5 meters.
Fly to (15, 0, -5): The drone moves 15 meters along the x-axis, keeping altitude constant.
Begin Circular Trajectory: The drone executes a counter-clockwise circular trajectory with a 15-meter radius, capturing images from its front RGB and depth cameras at each step.
Hover for 5 Seconds: After completing one revolution, the drone hovers for analysis.

## Stage 2: Marker Landing
Fly to (0, 0, -13): The drone descends to start the marker detection phase.
Fly to (0, 5, -13) and Hover for 5 Seconds: The drone hovers while recording the first marker.
Fly to (0, -5, -13) and Hover for 5 Seconds: The drone hovers and records the second marker.
Compare Markers by Depth: The drone evaluates the markers based on their depth values, selecting the closest one.
Lateral Approach to Selected Marker: The drone adjusts its path to directly approach the marker.
Landing: The drone uses Image-Based Visual Servoing to land by minimizing errors in the image space.

## Mathematical Details
### 1. Cylinder Size Estimation
To calculate the real-world size of the cylinder, we use the pixel dimensions of its bounding box and the object’s depth:
wm=wpx dot Z/fx , hm=hpx dot Z/fy
