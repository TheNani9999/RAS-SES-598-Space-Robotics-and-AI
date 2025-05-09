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

### Stage 1: Cylinder Estimation

1. Auto-arm and OFFBOARD mode
2. Take off vertically to (0, 0, -5)
3. Fly to (15, 0, -5)
4. Begin circular trajectory of radius 15 m, counter-clockwise
5. At each step, analyze `/drone/front_rgb` + `/drone/front_depth` for cylinders
6. After **one full revolution**, hover for 5 seconds

### Stage 2: Marker Landing 

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

To compute real-world width and height from bounding box:

```math
w_m = \frac{w_{px} \cdot Z}{f_x}, \quad
h_m = \frac{h_{px} \cdot Z}{f_y}
```
Where:

𝑤
𝑚
,
ℎ
𝑚 are the real-world width and height (meters),

𝑤
𝑝
𝑥
,
ℎ
𝑝
𝑥 are the pixel dimensions of the cylinder’s bounding box,


Z is the depth (distance from the camera),

𝑓
𝑥
,
𝑓
𝑦 are the focal lengths of the camera (in pixels).


### 2. Circle Flight Path

The drone follows:

```math
x = R \cos\theta, \quad y = R \sin\theta, \quad z = -5
```

Where:

𝑥
,
𝑦
are the coordinates on the circle,


R is the radius of the circle (meters),

𝜃 is the angular position in radians.


### 3. (Image-Based Visual Servoing) Control

#### Error Definition

```math
e_x = x_m \quad,
e_y = y_m \quad,
e_z = Z - Z_{\text{des}}
```
Where:

𝑒
𝑥
,
𝑒
𝑦
,
𝑒
𝑧 are the errors in the image space and altitude,

𝑥
𝑚
,
𝑦
𝑚 are the marker errors in the image (pixels),

Z is the current altitude,

𝑍
𝑑
𝑒
𝑠 is the desired altitude (usually 0, representing the ground level).

#### Control Gains

```math
K_{\text{LAT}} = 0.1; \quad K_{\text{ALT}} = 0.05; \quad \Delta t = 0.1\; \text{s}
```
Where:

𝐾
𝐿
𝐴
𝑇
  and 
𝐾
𝐴
𝐿
𝑇 are the control gains for lateral and altitude movements, respectively.

#### Velocity Commands

```math
v_x = -K_{\text{LAT}} \cdot e_x,
v_y = -K_{\text{LAT}} \cdot e_y,
v_z = -K_{\text{ALT}} \cdot e_z
```

#### Setpoint Update

```math
X_{\text{next}} = X + v_x \cdot \Delta t,
Y_{\text{next}} = Y + v_y \cdot \Delta t,
Z_{\text{next}} = Z + v_z \cdot \Delta t
```
Where:

𝑋
,
𝑌
,
𝑍 are the current positions,

𝑣
𝑥
,
𝑣
𝑦
,
𝑣
𝑧 are the velocity commands,

Δt is the time step.
#### Trigger Landing

```math
\sqrt{e_x^2 + e_y^2 + e_z^2} < 0.1 \Rightarrow \text{LAND}
```
