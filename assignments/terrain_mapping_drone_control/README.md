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
where
Bounding box and intrinsics:
```math
w_{px}, h_{px} are  the  real-world  width  and  height  (meters),
Z is the depth (distance from the camera),
f_x, f_y are the focal lengths of the camera (in pixels).
```
---

### 2. Circle Flight Path

The drone follows:

```math
x = R \cos\theta, \quad y = R \sin\theta, \quad z = -5
```

One full revolution is complete when:

```math
|\theta - \theta_0| \geq 2\pi
```

---

### 3. (Image-Based Visual Servoing) Control

#### Error Definition

```math
e_x = x_m \quad,
e_y = y_m \quad,
e_z = Z - Z_{\text{des}}
```
where
```math
(x_m, y_m) \quad \text{marker error in image space},
Z \quad \text{current drone altitude},
Z_{\text{des}} = 0 \quad \text{desired ground level}
```

#### Control Gains

```math
K_{\text{LAT}} = 0.1; \quad K_{\text{ALT}} = 0.05; \quad \Delta t = 0.1\; \text{s}
```

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

#### Trigger Landing

```math
\sqrt{e_x^2 + e_y^2 + e_z^2} < 0.1 \Rightarrow \text{LAND}
```
