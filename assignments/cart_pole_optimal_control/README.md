# Cart-Pole LQR Controller 
## Introduction
The Cart-Pole problem is a classic control system challenge, where an inverted pendulum is balanced on a moving cart. 
This system is inherently unstable, requiring continuous feedback control to maintain equilibrium.

This project implements a Linear Quadratic Regulator (LQR) controller to stabilize the system within ROS2, 
leveraging state-space representation and optimal control theory.

The controller is designed to minimize deviations in cart position and pole angle while ensuring efficient use of control force. 
Parameter tuning has been performed to optimize performance based on system stability, 
constraint satisfaction, and control efficiency.

## System Requirements
- Ubuntu 24.04.1
- ROS2 Jazzy
- cart_pole_optimal_control package
- numpy, scipy
  
## Installation
Clone the repository and build your workspace:
- cd ~/ros2_ws/src
- git clone <repo-url>
- cd ~/ros2_ws
- colcon build
- source install/setup.bash

## Running the Controller
To launch the controller, execute:
- ros2 run cart_pole_optimal_control cart_pole_lqr_controller
  
## State-Space Representation

The cart–pole system is modeled as a *linear time-invariant (LTI) system* with the state vector:

$$
x = \begin{bmatrix} 
x_c \\ 
\dot{x}_c \\ 
\theta \\ 
\dot{\theta} 
\end{bmatrix}
$$

Where:
- $x_c$ is the cart’s position.
- $\dot{x}_c$ is the cart’s velocity.
- $\theta$ is the pole’s angle from the vertical.
- $\dot{\theta}$ is the pole’s angular velocity.

The system is controlled by applying a force \( u \) to the cart, which affects the system dynamics.

## LQR Controller: The Role of Q and R
Q Matrix: State Penalty
The Q matrix determines how much each state deviation is penalized. 
A larger value in Q means that deviations in that particular state are penalized more heavily, 
forcing the controller to correct it faster.

## 1. System Parameters Updates

- The cart mass (M) was updated to 1.1 kg and the pole mass (m) to 0.95 kg, improving the realism of the model.

- The pole length (L) was adjusted to 1.1 m to reflect accurate physical constraints.

- The gravitational acceleration (g) remains at 9.81 m/s².

## 2. LQR Cost Matrix Tuning

- Q matrix was set to diag([100.0, 50.0, 100.0, 10.0]), increasing weight on cart position and pole angle.

- R matrix was set to [[0.01]], making the controller more aggressive in applying force.

## 3. Force Clipping for Physical Actuator Limits

- Control input (u) is now clipped between -15.0 and 15.0 N to prevent unrealistic force commands.

- This ensures the simulated system behaves within expected actuator capabilities.

## 4. State Estimation Smoothing

- Applied an exponential moving average filter to state updates: self.x = 0.85 * self.x + 0.15 * new_x.

- Helps reduce noise in joint state readings, leading to smoother control performance.

## 5. Reference Position Oscillation

- A small sinusoidal oscillation (0.5 * sin(t * 0.5)) was introduced in the desired cart position.

- This helps test the controller’s ability to stabilize the system dynamically.

### Initially, the Q matrix was:

- Q=diag([1.0,1.0,10.0,10.0])

### After tuning, the new Q matrix is:

- Q=diag([100.0,50.0,100.0,10.0])

## The rationale behind these changes: 
- Increased penalty on cart position (𝑄11=100.0) → Forces the cart to remain near its reference position.
- Higher weight on pole angle (𝑄33=100.0) → Stronger correction to keep the pole upright.
- Moderate weight on cart velocity (𝑄22=50.0) → Ensures smooth motion without excessive control effort.
- Pole angular velocity weight kept lower (𝑄44=10.0) → Reduces unnecessary oscillations.

## R Matrix: Control Effort Penalty
- The R matrix penalizes excessive control force. It is a scalar because there is only one control input (force on the cart).

- Initially:𝑅=0.1
- After tuning:𝑅=0.01

This allows the controller to apply stronger forces, leading to faster stabilization. 
The trade-off is that more aggressive control actions are allowed, but force saturation is prevented by clipping the control input to ±15N.

# ATTACHMENTS

https://github.com/user-attachments/assets/8b38949c-07bb-4559-acf2-4edca77acc9b

![CART POLE1](https://github.com/user-attachments/assets/4c9bca8a-2b69-4240-b6fe-2f99037d86ea)

![CART POLE2](https://github.com/user-attachments/assets/6fac5ac7-befa-4357-b867-12172c0eb04e)

![CART POLE3](https://github.com/user-attachments/assets/2dec1349-c131-43dd-97a2-20fda9352f40)


## Implementation Details
This project follows an incremental tuning approach, allowing real-time analysis and modifications in ROS2 and Gazebo.

## MATLAB Simulation and Graph Analysis
The MATLAB simulation confirms that the tuned LQR controller is stable and follows the expected behavior.

## Cart Position and Velocity
The cart initially moves due to the force required to stabilize the pole.
Settles near zero within ~2.5 seconds without overshooting.
The cart velocity peaks initially, then quickly dampens.

![CP 1](https://github.com/user-attachments/assets/4a85ed72-d41c-4f73-a249-10160b6ff675)

## Pole Angle and Angular Velocity
The pole starts with an initial angle deviation but rapidly stabilizes.
Minimal oscillations after the first second.
The angular velocity peaks but then damps smoothly.

![CP 2](https://github.com/user-attachments/assets/fea39f55-c0dd-4c2f-9e55-7dee2625abdc)

## Control Input (Force Applied)
The initial force is high (~10N), as expected for stabilization.
The force reduces as the system stabilizes.
Saturation limit of ±15N prevents excessive force application.

![CP 3](https://github.com/user-attachments/assets/1bf2ec08-0e32-4863-86c1-0af57137a6da)

## The Graph
The graph confirms that the controller is well-tuned:
Fast stabilization without excessive oscillations.
Minimal control effort after stabilization.
Physical force constraints respected.

 ![GRAPH SPACE AS 2](https://github.com/user-attachments/assets/09eed3c1-a508-4f9f-a0a5-1952724a210b)

## Future Optimizations: Bayesian Optimization
Currently, Q and R tuning is done manually, but this can be automated using Bayesian Optimization.

## Why Use Bayesian Optimization?
Automatically finds the best Q and R values.
Minimizes human effort in tuning.

## Implementation Plan
Define an optimization objective (e.g., minimizing settling time, control effort).
Use Bayesian Optimization to iteratively test different Q/R values.
Select the best-performing parameters.

# Key Findings and Improvements
## What’s Improved?
- Cart and pole stabilize faster with optimized Q and R matrices.
- Minimal oscillations ensure smooth system behavior.

# Final Thoughts
This project successfully implements, tunes, and validates an LQR controller for the Cart-Pole system. 
The performance is optimized, ensuring:
## Fast stabilization.
## Minimal oscillations.
## Efficient control effort.

## License
This work is licensed under a [Creative Commons Attribution 4.0 International License](http://creativecommons.org/licenses/by/4.0/).
[![Creative Commons License](https://i.creativecommons.org/l/by/4.0/88x31.png)](http://creativecommons.org/licenses/by/4.0/) 
