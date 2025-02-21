# Cart-Pole Optimal Control Assignment

[Watch the demo video](https://drive.google.com/file/d/1UEo88tqG-vV_pkRSoBF_-FWAlsZOLoIb/view?usp=sharing)
![image](https://github.com/user-attachments/assets/c8591475-3676-4cdf-8b4a-6539e5a2325f)

## Overview
This assignment challenges students to tune and analyze an LQR controller for a cart-pole system subject to earthquake disturbances. The goal is to maintain the pole's stability while keeping the cart within its physical constraints under external perturbations. The earthquake force generator in this assignment introduces students to simulating and controlling systems under seismic disturbances, which connects to the Virtual Shake Robot covered later in the course. The skills developed here in handling dynamic disturbances and maintaining system stability will be useful for optimal control of space robots, such as Lunar landers or orbital debris removal robots.

## System Description
The assignment is based on the problem formalism here: https://underactuated.mit.edu/acrobot.html#cart_pole
### Physical Setup
- Inverted pendulum mounted on a cart
- Cart traversal range: ±2.5m (total range: 5m)
- Pole length: 1m
- Cart mass: 1.0 kg
- Pole mass: 1.0 kg

### Disturbance Generator
The system includes an earthquake force generator that introduces external disturbances:
- Generates continuous, earthquake-like forces using superposition of sine waves
- Base amplitude: 15.0N (default setting)
- Frequency range: 0.5-4.0 Hz (default setting)
- Random variations in amplitude and phase
- Additional Gaussian noise

## Assignment Objectives

### Core Requirements
1. Analyze and tune the provided LQR controller to:
   - Maintain the pendulum in an upright position
   - Keep the cart within its ±2.5m physical limits
   - Achieve stable operation under earthquake disturbances
2. Document your LQR tuning approach:
   - Analysis of the existing Q and R matrices
   - Justification for any tuning changes made
   - Analysis of performance trade-offs
   - Experimental results and observations
3. Analyze system performance:
   - Duration of stable operation
   - Maximum cart displacement
   - Pendulum angle deviation
   - Control effort analysis

### Learning Outcomes
- Understanding of LQR control parameters and their effects
- Experience with competing control objectives
- Analysis of system behavior under disturbances
- Practical experience with ROS2 and Gazebo simulation

### Extra Credit Options
Students can implement reinforcement learning for extra credit (up to 30 points):

1. Reinforcement Learning Implementation:
   - Implement a basic DQN (Deep Q-Network) controller
   - Train the agent to stabilize the pendulum
   - Compare performance with the LQR controller
   - Document training process and results
   - Create training progress visualizations
   - Analyze and compare performance with LQR

## Implementation

### Controller Description
The package includes a complete LQR controller implementation (`lqr_controller.py`) with the following features:
- State feedback control
- Configurable Q and R matrices
- Real-time force command generation
- State estimation and processing

Current default parameters:
```python
# State cost matrix Q (default values)
Q = np.diag([1.0, 1.0, 10.0, 10.0])  # [x, x_dot, theta, theta_dot]

# Control cost R (default value)
R = np.array([[0.1]])  # Control effort cost
```

### Earthquake Disturbance
The earthquake generator (`earthquake_force_generator.py`) provides realistic disturbances:
- Configurable through ROS2 parameters
- Default settings:
  ```python
  parameters=[{
      'base_amplitude': 15.0,    # Strong force amplitude (N)
      'frequency_range': [0.5, 4.0],  # Wide frequency range (Hz)
      'update_rate': 50.0  # Update rate (Hz)
  }]
  ```

## Getting Started

### Prerequisites
- ROS2 Humble or Jazzy
- Gazebo Garden
- Python 3.8+
- Required Python packages: numpy, scipy

#### Installation Commands
```bash
# Set ROS_DISTRO as per your configuration
export ROS_DISTRO=humble

# Install ROS2 packages
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-interfaces \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-rviz2

# Install Python dependencies
pip3 install numpy scipy control
```

### Repository Setup

#### If you already have a fork of the course repository:
```bash
# Navigate to your local copy of the repository
cd ~/RAS-SES-598-Space-Robotics-and-AI

# Add the original repository as upstream (if not already done)
git remote add upstream https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI.git

# Fetch the latest changes from upstream
git fetch upstream

# Checkout your main branch
git checkout main

# Merge upstream changes
git merge upstream/main

# Push the updates to your fork
git push origin main
```

#### If you don't have a fork yet:
1. Fork the course repository:
   - Visit: https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI
   - Click "Fork" in the top-right corner
   - Select your GitHub account as the destination

2. Clone your fork:
```bash
cd ~/
git clone https://github.com/YOUR_USERNAME/RAS-SES-598-Space-Robotics-and-AI.git
```

### Create Symlink to ROS2 Workspace
```bash
# Create symlink in your ROS2 workspace
cd ~/ros2_ws/src
ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/cart_pole_optimal_control .
```

### Building and Running
```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select cart_pole_optimal_control --symlink-install

# Source the workspace
source install/setup.bash

# Launch the simulation with visualization
ros2 launch cart_pole_optimal_control cart_pole_rviz.launch.py
```

This will start:
- Gazebo simulation (headless mode)
- RViz visualization showing:
  * Cart-pole system
  * Force arrows (control and disturbance forces)
  * TF frames for system state
- LQR controller
- Earthquake force generator
- Force visualizer

### Visualization Features
The RViz view provides a side perspective of the cart-pole system with:

#### Force Arrows
Two types of forces are visualized:
1. Control Forces (at cart level):
   - Red arrows: Positive control force (right)
   - Blue arrows: Negative control force (left)

2. Earthquake Disturbances (above cart):
   - Orange arrows: Positive disturbance (right)
   - Purple arrows: Negative disturbance (left)

Arrow lengths are proportional to force magnitudes.

## Analysis Requirements

### Performance Metrics
Students should analyze:
1. Stability Metrics:
   - Maximum pole angle deviation
   - RMS cart position error
   - Peak control force used
   - Recovery time after disturbances

2. System Constraints:
   - Cart position limit: ±2.5m
   - Control rate: 50Hz
   - Pole angle stability
   - Control effort efficiency

### Analysis Guidelines
1. Baseline Performance:
   - Document system behavior with default parameters
   - Identify key performance bottlenecks
   - Analyze disturbance effects

2. Parameter Effects:
   - Analyze how Q matrix weights affect different states
   - Study R value's impact on control aggressiveness
   - Document trade-offs between objectives

3. Disturbance Response:
   - Characterize system response to different disturbance frequencies
   - Analyze recovery behavior
   - Study control effort distribution

## Evaluation Criteria
### Core Assignment (100 points)
1. Analysis Quality (40 points)
   - Depth of parameter analysis
   - Quality of performance metrics
   - Understanding of system behavior

2. Performance Results (30 points)
   - Stability under disturbances
   - Constraint satisfaction
   - Control efficiency

3. Documentation (30 points)
   - Clear analysis presentation
   - Quality of data and plots
   - Thoroughness of discussion

### Extra Credit (up to 30 points)
- Reinforcement Learning Implementation (30 points)

## Tips for Success
1. Start with understanding the existing controller behavior
2. Document baseline performance thoroughly
3. Make systematic parameter adjustments
4. Keep detailed records of all tests
5. Focus on understanding trade-offs
6. Use visualizations effectively

## Submission Requirements
1. Technical report including:
   - Analysis of controller behavior
   - Performance data and plots
   - Discussion of findings
2. Video demonstration of system performance
3. Any additional analysis tools or visualizations created
   

Cart-Pole LQR Controller – ROS2 Implementation

Introduction
The Cart-Pole problem is a classic control system challenge, where an inverted pendulum is balanced on a moving cart. This system is inherently unstable, requiring continuous feedback control to maintain equilibrium.

This project implements a Linear Quadratic Regulator (LQR) controller to stabilize the system within ROS2, leveraging state-space representation and optimal control theory.

The controller is designed to minimize deviations in cart position and pole angle while ensuring efficient use of control force. Parameter tuning has been performed to optimize performance based on system stability, constraint satisfaction, and control efficiency.

State-Space Representation
The system is modeled as a linear time-invariant (LTI) system with the state vector:
x= [xc,xc˙,θ,θ˙]
where:𝑥𝑐 - cart's position
      𝑥˙𝑐- Cart's Velocity
      θ  - Pole's angle from the vertical 
      θ˙ - Pole's angular velocity
The control input 𝑢 is the force applied to the cart, which directly affects system dynamics.

LQR Controller: The Role of Q and R
Q Matrix: State Penalty
The Q matrix determines how much each state deviation is penalized. A larger value in Q means that deviations in that particular state are penalized more heavily, forcing the controller to correct it faster.

Initially, the Q matrix was:

Q=diag([1.0,1.0,10.0,10.0])
After tuning, the new Q matrix is:

Q=diag([100.0,50.0,100.0,10.0])
The rationale behind these changes:

Increased penalty on cart position (𝑄11=100.0)→ Forces the cart to remain near its reference position.
Higher weight on pole angle (𝑄33=100.0) → Stronger correction to keep the pole upright.
Moderate weight on cart velocity (𝑄22=50.0) → Ensures smooth motion without excessive control effort.
Pole angular velocity weight kept lower (𝑄44=10.0) → Reduces unnecessary oscillations.

R Matrix: Control Effort Penalty
The R matrix penalizes excessive control force. It is a scalar because there is only one control input (force on the cart).

Initially:𝑅=0.1

After tuning:𝑅=0.01
This allows the controller to apply stronger forces, leading to faster stabilization. The trade-off is that more aggressive control actions are allowed, but force saturation is prevented by clipping the control input to ±15N.

Implementation Details
This project follows an incremental tuning approach, allowing real-time analysis and modifications in ROS2 and Gazebo.

Run the Controller in ROS2
1️⃣ Build and Source the ROS2 Package

colcon build
source install/setup.bash

2️⃣ Launch the Cart-Pole Simulation

ros2 launch cart_pole_gazebo cart_pole.launch.py

3️⃣ Run the LQR Controller

ros2 run cart_pole_control cart_pole_lqr_controller

MATLAB Simulation and Graph Analysis
The MATLAB simulation confirms that the tuned LQR controller is stable and follows the expected behavior.

Cart Position and Velocity
The cart initially moves due to the force required to stabilize the pole.
Settles near zero within ~2.5 seconds without overshooting.
The cart velocity peaks initially, then quickly dampens.

Pole Angle and Angular Velocity
The pole starts with an initial angle deviation but rapidly stabilizes.
Minimal oscillations after the first second.
The angular velocity peaks but then damps smoothly.

Control Input (Force Applied)
The initial force is high (~10N), as expected for stabilization.
The force reduces as the system stabilizes.
Saturation limit of ±15N prevents excessive force application.

Is the Graph Correct?
Yes, the graph confirms that the controller is well-tuned:
Fast stabilization without excessive oscillations.
Minimal control effort after stabilization.
Physical force constraints respected.

Future Optimizations: Bayesian Optimization
Currently, Q and R tuning is done manually, but this can be automated using Bayesian Optimization.

Why Use Bayesian Optimization?
Automatically finds the best Q and R values.
Minimizes human effort in tuning.
Can be extended to Reinforcement Learning for adaptive control.

Implementation Plan
Define an optimization objective (e.g., minimizing settling time, control effort).
Use Bayesian Optimization to iteratively test different Q/R values.
Select the best-performing parameters.

Comparison of Different Q and R Values
To understand the effect of different Q and R values, multipliers were introduced.
Q x 1.5 and R x 0.8
Faster stabilization, but higher force usage.
More aggressive control.
Q x 0.8 and R x 1.5
Slower stabilization, less control effort.
More stable, but takes longer to correct deviations.
These trade-offs demonstrate the importance of proper tuning.

Key Findings and Improvements
What’s Improved?
Cart and pole stabilize faster with optimized Q and R matrices.
Minimal oscillations ensure smooth system behavior.
MATLAB simulation validates correctness.

Next Steps
Implement Bayesian Optimization for auto-tuning.
Explore Reinforcement Learning for adaptive control.
Final Thoughts
This project successfully implements, tunes, and validates an LQR controller for the Cart-Pole system. The performance is optimized, ensuring:

Fast stabilization.
Minimal oscillations.
Efficient control effort.

## License
This work is licensed under a [Creative Commons Attribution 4.0 International License](http://creativecommons.org/licenses/by/4.0/).
[![Creative Commons License](https://i.creativecommons.org/l/by/4.0/88x31.png)](http://creativecommons.org/licenses/by/4.0/) 
