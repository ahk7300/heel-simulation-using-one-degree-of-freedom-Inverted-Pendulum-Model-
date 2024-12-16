***Heel Support Dynamics in Ankle Joint Using Inverted Pendulum Modeling***
This project models the human ankle as a one-degree-of-freedom inverted pendulum to analyze the effects of heel support on oscillation frequency and angular velocity. The findings provide insights for rehabilitation, prosthetics, and sports biomechanics.

**Project Overview**
The human ankle plays a vital role in maintaining balance and facilitating smooth gait transitions. This project investigates how heel support impacts the dynamics of ankle movement, modeled as an inverted pendulum. Simulations compare two conditions:

With Heel Support: Shortened effective pendulum length.
Without Heel Support: Full pendulum length.
The study explores the biomechanical implications of these changes on natural frequency, oscillation speed, and angular velocity.

**Key Features**

**Biomechanics Simulation:**
Modeled the ankle joint as an inverted pendulum with one degree of freedom.
Simulated dynamic behavior using MATLAB’s ODE45 solver.

**Two Conditions:**
Heel Support: Shortens pendulum length, increasing oscillation frequency.
No Heel Support: Full pendulum length, leading to slower oscillations.
Graphical Analysis:
Angular displacement and velocity over time for both conditions.
Visual comparison of oscillation patterns.
Applications
Rehabilitation: Design heel-assisted therapies for gait training.
Prosthetics: Optimize ankle-foot prostheses to mimic natural dynamics.
Sports Biomechanics: Enhance athletic performance through customized footwear.

**Methodology**
Parameters:
Gravitational acceleration: 9.81 m/s².
Pendulum length:
With Heel Support: 95% of the original length.
Without Heel Support: Full length.
Initial angular displacement: ±10°.
Simulation:
MATLAB’s ODE45 solver used for numerical analysis.
Input: Gravitational force, pendulum length, initial conditions.
Output: Time-series data of angular displacement and velocity.
Analysis:
Compared the natural frequency and oscillation speed.
Visualized results through angular displacement and velocity graphs.
Results
Faster Oscillations with Heel Support:
Shortened pendulum length increases natural frequency.
Higher angular velocity observed due to faster oscillations.
Periodic Sinusoidal Motion:
Both conditions exhibit characteristic pendulum dynamics.
Clear differences in frequency and velocity peaks.
Future Directions
Experimental Validation:
Conduct studies on human subjects to validate findings.
Enhanced Models:
Extend to multi-degree-of-freedom systems for greater realism.
Broader Applications:
Explore custom footwear and prosthetics for various mobility needs.
Authors and Acknowledgments
Authors: Ali Haider Khan, Prof. James Patton PhD, Prof. Wu Ming PhD.
Acknowledgments: Special thanks to Dr. Patton and Prof. Ming for their guidance and feedback, and to peers and family for their support.
How to Use
Software Requirements:
MATLAB (R2021 or later).
Running Simulations:
Load the provided MATLAB code.
Input parameters for heel/no-heel conditions.
Analyze the output graphs for angular displacement and velocity.
References
Garcia-Gonzalez et al., Effects of Biomechanical Heel Support on Human Locomotion (2023).
Smith and Clarke, Nonlinear Dynamics of Inverted Pendulum Systems (2014).
Kim et al., Biomechanical Implications of Ankle Dynamics During Gait (2021).
Let 
