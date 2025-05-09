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

## Mission Overview

### Stage 1: Cylinder Estimation (Autonomous Circle Flight)

1. Auto-arm and OFFBOARD mode
2. Take off vertically to (0, 0, -5)
3. Fly to (15, 0, -5)
4. Begin circular trajectory of radius 15 m, counter-clockwise
5. At each step, analyze `/drone/front_rgb` + `/drone/front_depth` for cylinders
6. After **one full revolution**, hover for 5 seconds

### Stage 2: Marker Landing (IBVS)

1. Fly to (0, 0, -13)
2. Fly to (0, 5, -13), hover for 5 s, record marker sample
3. Return to center
4. Fly to (0, -5, -13), hover for 5 s, record second marker
5. Compare both markers by depth → choose closest
6. Laterally approach selected marker
7. Perform **IBVS landing** using marker pixel offset and drone altitude

---

## Mathematical Details
### 1. Cylinder Size Estimation
To calculate the real-world size of the cylinder, we use the pixel dimensions of its bounding box and the object’s depth:
wm=wpx dot Z/fx , hm=hpx dot Z/fy
