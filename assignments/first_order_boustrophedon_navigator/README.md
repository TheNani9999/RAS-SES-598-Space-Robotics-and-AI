Parameter Tuning and Performance Analysis

1. PD Controller Parameters:
In this project, the following PD controller parameters were tuned to improve the performance of the boustrophedon
pattern execution:
 Kp_linear (Proportional Gain for Linear Velocity):
 Initial Value:10.0
 Final Value:8.0
 Reason for Change: The initial Kp_linear was too high, resulting in an overly aggressive response that caused the
 robot to overshoot and veer off-course. By lowering the value to 8.0, the robot exhibited smoother linear motion with
 less overshooting.

 Kd_linear (Derivative Gain for Linear Velocity):
 Initial Value:0.1
 Final Value:0.05
 Reason for Change:The derivative term was originally too large, causing some oscillation during linear movement.
 Reducing Kd_linear helped smooth the velocity profile and eliminated jittering, especially when the robot was moving
 in a straight line.

 Kp_angular (Proportional Gain for Angular Velocity):
 Initial Value:5.0
 Final Value:7.0
 Reason for Change:The initial angular gain was too low, causing slow turns and inaccurate cornering. Increasing
 Kp_angular helped the robot turn more precisely, reducing cornering errors and improving path adherence.

 Kd_angular (Derivative Gain for Angular Velocity):
 Initial Value:0.2
 Final Value:0.000
 Reason for Change: A derivative gain of 0.2 led to oscillations during cornering, making the motion less smooth.
 Setting Kd_angular to 0.000 eliminated the oscillations, resulting in cleaner turns with minimal cross-track errors.

2. Boustraphedon Pattern Parameters:
The following pattern parameters were optimized to improve the efficiency of the boustrophedon survey:
Spacing Between Lines:
Initial Value:1.0
Final Value:0.5
Reason for Change: Reducing the spacing between lines allowed the robot to achieve higher coverage while
maintaining a more efficient and accurate survey pattern. This change helped the robot cover the area more
effectively without excessive overlap.

3. Performance Metrics:
Average Cross-Track Error:0.15 units
Maximum Cross-Track Error:0.45 units
Smoothness of Motion:The robot now follows a smooth path with less jerkiness, especially during straight-line
movements.
Cornering Performance:The robot now successfully navigates corners with minimal deviation from the intended path.

4. Analysis and Tuning Process:
The tuning process involved using rqt_reconfigure to adjust the PD controller parameters in real-time. Key
observations during tuning:
Linear Velocity:Reducing the proportional gain (Kp_linear) and the derivative gain (Kd_linear) significantly improved
the smoothness of linear motion and reduced the cross-track error
Angular Velocity:Increasing the proportional gain (Kp_angular) and eliminating the derivative gain (Kd_angular)
improved the robot’s cornering performance, allowing the robot to follow turns more accurately without overshooting.
Pattern Efficiency:The spacing between lines was reduced to ensure better coverage of the area and more efficient
use of the robot's path. This helped maintain consistent and optimal performance across the entire survey area.

5. Plots:
Cross-Track Error Over Time
Trajectory Plot
Velocity Profiles

Methodology Used for Tuning

Parameter Adjustment: The PD controller parameters were adjusted iteratively using rqt_reconfigure, allowing
real-time changes to the gains.
Observations: The robot’s performance was evaluated after each parameter change, focusing on reducing
cross-track error and improving smoothness.
Metrics-Based Analysis: Cross-track error, smoothness of motion, and cornering accuracy were used as key
performance indicators.
Fine-Tuning: Parameters were incrementally refined to balance responsiveness and stability.

Challenges Encountered and Solutions

1: Oscillations during straight-line motion due to high Kp_linear and Kd_linear.
Solution: Gradually reduced both gains until smooth motion was achieved.
2: Poor cornering accuracy due to low Kp_angular and oscillations caused by Kd_angular.
Solution: Increased Kp_angular for better turning precision and set Kd_angular to 0.000 to eliminate oscillations.
3: Inconsistent area coverage due to large spacing between lines.
Solution: Reduced spacing to 0.5 meters for optimal coverage.

Comparison of Different Parameter Sets

Initial Parameters: High gains led to aggressive, unstable responses with high cross-track error and oscillations.
Final Parameters: Balanced gains provided smoother motion, precise turns, and consistent area coverage with
minimal cross-track error.
