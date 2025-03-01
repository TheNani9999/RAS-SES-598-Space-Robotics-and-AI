
   

# Cart-Pole LQR Controller – ROS2 Implementation
## Introduction
The Cart-Pole problem is a classic control system challenge, where an inverted pendulum is balanced on a moving cart. 
This system is inherently unstable, requiring continuous feedback control to maintain equilibrium.

This project implements a Linear Quadratic Regulator (LQR) controller to stabilize the system within ROS2, 
leveraging state-space representation and optimal control theory.

The controller is designed to minimize deviations in cart position and pole angle while ensuring efficient use of control force. 
Parameter tuning has been performed to optimize performance based on system stability, 
constraint satisfaction, and control efficiency.

## State-Space Representation
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

### Initially, the Q matrix was:

Q=diag([1.0,1.0,10.0,10.0])

### After tuning, the new Q matrix is:

Q=diag([100.0,50.0,100.0,10.0])

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

## Implementation Details
This project follows an incremental tuning approach, allowing real-time analysis and modifications in ROS2 and Gazebo.

## Run the Controller in ROS2

## Build and Source the ROS2 Package

- cd ~/ros2_ws
- build
- source install/setup.bash

## Run the LQR Controller

- ros2 run cart_pole_control cart_pole_lqr_controller

## MATLAB Simulation and Graph Analysis
The MATLAB simulation confirms that the tuned LQR controller is stable and follows the expected behavior.

## Cart Position and Velocity
The cart initially moves due to the force required to stabilize the pole.
Settles near zero within ~2.5 seconds without overshooting.
The cart velocity peaks initially, then quickly dampens.

## Pole Angle and Angular Velocity
The pole starts with an initial angle deviation but rapidly stabilizes.
Minimal oscillations after the first second.
The angular velocity peaks but then damps smoothly.

## Control Input (Force Applied)
The initial force is high (~10N), as expected for stabilization.
The force reduces as the system stabilizes.
Saturation limit of ±15N prevents excessive force application.

## Is the Graph Correct?
Yes, the graph confirms that the controller is well-tuned:
Fast stabilization without excessive oscillations.
Minimal control effort after stabilization.
Physical force constraints respected.

## Future Optimizations: Bayesian Optimization
Currently, Q and R tuning is done manually, but this can be automated using Bayesian Optimization.

## Why Use Bayesian Optimization?
Automatically finds the best Q and R values.
Minimizes human effort in tuning.
Can be extended to Reinforcement Learning for adaptive control.

## Implementation Plan
Define an optimization objective (e.g., minimizing settling time, control effort).
Use Bayesian Optimization to iteratively test different Q/R values.
Select the best-performing parameters.

## Comparison of Different Q and R Values
### To understand the effect of different Q and R values, multipliers were introduced.

Q x 1.5 and R x 0.8
Faster stabilization, but higher force usage.
More aggressive control.

Q x 0.8 and R x 1.5
Slower stabilization, less control effort.
More stable, but takes longer to correct deviations.
These trade-offs demonstrate the importance of proper tuning.

# Key Findings and Improvements
## What’s Improved?
- Cart and pole stabilize faster with optimized Q and R matrices.
- Minimal oscillations ensure smooth system behavior.
- MATLAB simulation validates correctness.

# Final Thoughts
This project successfully implements, tunes, and validates an LQR controller for the Cart-Pole system. 
The performance is optimized, ensuring:
### Fast stabilization.
### Minimal oscillations.
### Efficient control effort.

## License
This work is licensed under a [Creative Commons Attribution 4.0 International License](http://creativecommons.org/licenses/by/4.0/).
[![Creative Commons License](https://i.creativecommons.org/l/by/4.0/88x31.png)](http://creativecommons.org/licenses/by/4.0/) 
