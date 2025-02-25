# PD Controller For 2D Quadrotor
This project involves implementing a Proportional-Derivative (PD) control system for a two-dimensional (2D) quadrotor. The objective is to stabilize the quadrotor's position ensuring smooth motion and maintaining the desired trajectory. The PD controller computes corrective actions based on the desired acceleration, position error (the difference between desired and current positions), velocity error (the difference between desired and current velocities), and roll angle error (the difference between desired and current roll angles), allowing for precise control.

## Table of Contents
- [Introduction](#introduction)
   - [Technical Details](#technical-details)
   - [System Model](#system-model)
   - [PD Controller](#pd-controller)
- [Features](#features)
- [Requirements](#requirements)
- [Usage](#usage)
- [File Structure](#file-structure)
- [Results](#results)
- [Future Work](#future-work)
- [License](#license)
- [Acknowledgments](#acknowledgments)
- [References](#references)

## Introduction

The goal of this project is to gain familiarity with quadrotor dynamics and implement a Proportional-Derivative (PD) controller. The PD controller is designed to stabilize the motion of the quadrotor by adjusting the thrust and moment to achieve and maintain a specific trajectory.

### Technical Details  

The physical properties of the quadrotor are:
- Mass ($m$) : 0.18($kg$)
- Lenght of arm ($L$) : 0.086($m$)
- Moment of Inertia ($I_{xx}$) : 0.00025 ($kgm^2$)
  
### System Model

<div align="center">

![Figure 1: Quadrotor Schematic](Quadrotor_Schematic.png)

**Figure 1**: Planar quadrotor model and the coordinate systems.

</div>

#### Coordinate Systems

The coordinate systems and free body diagram for the planar model of a quadrotor are shown in **Figure 1**. The inertial frame, $\mathcal{A}$, is defined by axes $a_2$ and $a_3$. The body frame, $\mathcal{B}$, is attached to the center of mass of the quadrotor with $b_2$ coinciding with the preferred forward direction and $b_3$ perpendicular to the rotors pointing vertically up (see **Figure 1**).

#### Dynamics


For a quadrotor modeled in the $Y$ − $Z$ plane, its orientation is defined by a roll angle, $\phi$. It is assumed that its pitch and yaw angles are 0. You will need the rotation matrix for transforming components of vectors in $\mathcal{B}$ to components of vectors in $\mathcal{A}$:

$$
{}^\mathcal{A}[R]_\mathcal{B} = 
\begin{bmatrix} \cos(\phi) & -\sin(\phi) \\ 
\sin(\phi) & \cos(\phi) 
\end{bmatrix} 
\quad (1)
$$

We will denote the components of angular velocity of the robot in the body frame by $\dot{\phi}$:

$$
{}^\mathcal{A}\omega_\mathcal{B} = \dot{\phi}
$$

In the planar model of the quadrotor, we only consider the thrust force on two of the rotors. The quadrotor has two inputs: the thrust force ($u_1$) and the moment ($u_2$). $u_1$ is the sum of the thrusts at each rotor

$$
u_1 = \sum_{i=1}^2 F_i,
$$

while $u_2$ is proportional to the difference between the thrusts of two rotors:

$$
u_2 = L(F_1 - F_2)
$$

Here, $L$ is the arm length of the quadrotor.
Let $\mathbf{r} = [y, z]^T$ denote the position vector of the planar quadrotor in $\mathcal{A}$. The forces on the system are gravity, in the $−a_3$ direction, and the thrust force, in the $b_3$ direction. Hence, by Newton’s Equations of Motion,

$$
m\ddot{\mathbf{r}} = m[\ddot{y}; \ddot{z}] = [0; -mg] + {}^\mathcal{A}[R]_\mathcal{B}[0; u_1] = [0; -mg] + [-u_1\sin(\phi); u_1\cos(\phi)] \quad (2)
$$

The angular acceleration is determined by Euler's equation of motion:

$$
I_{xx}\ddot{\phi} = L(F_1 - F_2) = u_2
$$

As a result, the system model can be written as:

$$
[\ddot{y}; \ddot{z}; \ddot{\phi}] = [0; -g; 0] + [-\frac{1}{m} \sin(\phi), 0; \frac{1}{m} \cos(\phi), 0; 0, \frac{1}{I_{xx}}] [u_1; u_2] \quad (3)
$$

### PD Controller

#### Linearisation

The dynamic model of the quadrotor (Eq. 3) is nonlinear. However, a PD controller is designed for a linear system. To use a linear controller for this nonlinear system, we first linearize the equation of motions about an equilibrium configuration.

In the case of the quadrotor, the equilibrium configuration is the hover configuration at any arbitrary position $y_0$, $z_0$, with zero roll angle. The corresponding thrust force needed to hover at this configuration is exactly $mg$, while the moment must be zero. Explicitly, the values of the related variables at the hover configuration are

$$
y_0,z_0,\phi_0 = 0,u_{1,0} = mg,u_{2,0} = 0
$$

To linearize the dynamics, we replace all non-linear function of the state and control variables with their first order Taylor approximations at the equilibrium location. In this case, the non-linear functions are $sin(\phi)$ and $cos(\phi)$. Near $\phi = 0$, $sin(\phi) = \phi$ and $cos(\phi) = 1$

$$
\ddot{y} = -g\phi
$$

$$
\ddot{z} = -g + u_1/m
$$

$$
\ddot{\phi} = u_2/I_{xx}
$$

Let $r$ denote a state variable, either $y$, $z$ or $\phi$. We can find the commanded acceleration of that state, $\ddot{r}$, corresponding to a (PD) controller as follows. Define the position and velocity errors as

$$
e_{p} = r_{T}(t) - r
$$

$$
e_{v} = \dot{r}_{T}(t) - \dot{r}
$$

We want error to satisfy the following differential equation, which will result in convergence of the error for some value of $k_p$ and $k_d$.

$$
(\ddot{r_T}(t) - \ddot{r_c}) + k_pe_p + k_de_v = 0 \quad(4)
$$

From this, we can see that

$$
\ddot{r_c} = \ddot{r_T}(t) + k_pe_p + k_de_v = 0 \quad(5)
$$

where $k_p$ and $k_d$ are are proportional and derivative gains respectively.

As a result, the inputs $u_1$, $u_2$, can be derived as:

$$ 
u_1 = mg + m\ddot{z}_c \quad(6)
$$

$$ 
u_2 = I_{xx}\ddot{\phi}_c \quad(7)
$$

$$
\phi_c = -\frac{\ddot{y}_c}{g} \quad(8)
$$

#### Hover Contorller

Hovering is the special case of which the desired position is constant and the desired roll is zero. Which means $\mathbf{r}_T(t) = \mathbf{r}_0 = [y_0, z_0]^T, \phi_T(t) = \phi_0, \dot{\mathbf{r}}_T(t) = \ddot{\mathbf{r}}_T(t) = 0$ the inputs $u_1$, $u_2$ and $\phi_c$ can be calculated using Equations 6-8.

#### Trajectory Controller

For trajectory following, given the desired trajectories for each state and their derivatives, $r_T(t)$, $\dot{r_T}(t)$, $\ddot{r_T}(t)$, the inputs $u_1$, $u_2$ and $\phi_c$ can be calculated using Equations 6-8.

In this project, I have implemented a custom PD controller to control the trajectory of quadrotor and tuned $k_p$ and $k_d$ values for four trajectory cases:
1. **Simple Line**
2. **Sine Wave**
3. **Diamond Shape**
4. **Step Input**

## Features  

- **PD Controller Implementation**:  
  Implements a custom Proportional-Derivative (PD) controller to manage the quadrotor's position in trajectory.  

- **Simulation Environment**:  
  Provides a realistic simulation of the quadrotor's dynamics under the control of the PD system.  

- **Graphical User Interface (GUI)**:  
  Includes an interactive GUI for:
  - Simulating the quadrotor's motion.
  - Visualizing the controller's response over time.

## Requirements

- MATLAB (preferably R2018b or later)

## Usage

Clone this repository and run the `line_simulation.m` file for line trajectory, `sine_wave_simulation.m` file for sine wave trajectory, `diamond_shape_simulation.m` file for diamond shape trajectory and `step_input_simulation.m` file for step input trajectory.

## **File Structure**

The project consists of the following MATLAB functions:

- **`simulation_2d.m`**:  
   The main function that initiates the 2D quadrotor simulation. It connects the desired trajectory, PD controller, and dynamics, and visualizes the quadrotor’s planar motion and controller performance.

- **`controller.m`**:  
   Implements a Proportional-Derivative (PD) controller to stabilize the 2D quadrotor. The controller computes the required thrust and moment inputs based on position, velocity, and roll angle errors, ensuring the quadrotor follows the desired trajectory.

- **`line_simulation.m`**:  
   Executes a simulation where the quadrotor follows a straight-line trajectory. It demonstrates how the PD controller maintains a constant linear path.

- **`sine_wave_simulation.m`**:  
   Runs a simulation in which the quadrotor tracks a sine wave trajectory. This tests the controller’s ability to manage continuously varying reference positions.

- **`diamond_shape_simulation.m`**:  
   Simulates the quadrotor following a diamond-shaped trajectory. It highlights the controller’s performance during rapid changes in direction and more complex motion patterns.

- **`step_input_simulation.m`**:  
   Simulates the quadrotor’s response to a step input trajectory. This is useful for evaluating the transient response of the PD controller when the desired state undergoes sudden changes.

- **`libs/`**:  
   Contains helper functions for the simulation:  
   - **`QuadPlot.m`**:  
     Visualizes the quadrotor's position and trajectory over time, including motor positions and historical movement.  
   - **`QuatToRot.m`**:  
     Converts a unit quaternion into its corresponding $3 \times 3$ rotation matrix. Useful for orientation computations.  
   - **`quad_pos.m`**:  
     Calculates the quadrotor's position in the world frame based on its state.  
   - **`simStateToQuadState.m`**:  
     Converts simulation state data into the 13-element state vector format:  
     $[x, y, z, \dot{x}, \dot{y}, \dot{z}, q_w, q_x, q_y, q_z, p, q, r]$.  
   - **`sys_eom.m`**:  
     Defines the equations of motion (EOM) for the quadrotor system. For 1D motion, it simplifies to:  
     $\ddot{z} = \frac{u}{m} - g$  
     where $u$ is the thrust generated by the controller.  
   - **`sys_params.m`**:  
     Sets system parameters, such as the quadrotor's mass $m$, gravitational acceleration $g$, arm lenght $l$, moment of inertia about the x-axis $I_{xx}$, minimum thrust input $u_{min}$ and maximum thrust input $u_{max}$. 
   

- **`trajectories/`**:  
   Contains trajectory generation functions used to define the desired reference trajectories:  
   - **`traj_diamond.m`**:  
     Generates a diamond-shaped trajectory for the simulation.  
   - **`traj_line.m`**:  
     Generates a straight-line trajectory.  
   - **`traj_sine.m`**:  
     Generates a sine wave trajectory.  
   - **`traj_step.m`**:  
     Generates a step input trajectory.

## Results

### Quadrotor Simulation for Line Trajectory and Controller Response
![Quadrotor Simulation for Line Trajectory and Controller Response](traj_line.gif)

### Quadrotor Simulation for Sine Wave Trajectory and Controller Response
![Quadrotor Simulation for Sine Wave Trajectory and Controller Response](traj_sine.gif)

### Quadrotor Simulation for Diamond Shape Trajectory and Controller Response
![Quadrotor Simulation for Diamond Shape Trajectory and Controller Response](traj_diamond.gif)

### Quadrotor Simulation for Step Input Trajectory and Controller Response
![Quadrotor Simulation for Step Input Trajectory and Controller Response](traj_step.gif)

## Future Work  

The current implementation effectively stabilizes the 2D quadrotor and performs basic position control. However, there are several areas where the system can be enhanced while remaining within the 2D motion framework. These improvements aim to refine control precision, robustness, and adaptability under realistic conditions:  

### 1. **Adaptive Gain Tuning**  
   The fixed proportional ($K_p$) and derivative ($K_d$) gains may not be optimal under varying conditions. Adaptive gain tuning can dynamically adjust these parameters to:  
   - Enhance performance under changing conditions, such as varying payloads or external disturbances.  
   - Reduce overshoot and oscillations for different types of trajectories.  
   Real-time optimization techniques, such as Recursive Least Squares (RLS) or model-based adaptive control, can be employed to achieve this.  

### 2. **Robustness Against Disturbances and Noise**  
   Real-world scenarios often introduce uncertainties, such as external disturbances or sensor noise. To make the system more robust:  
   - **Disturbance Rejection**: Design disturbance observers or use feedforward compensation to counteract known disturbances, such as wind or persistent forces.  
   - **Sensor Noise Handling**: Implement low-pass or Kalman filters to reduce the impact of noise on velocity and position measurements. This is especially important when estimating derivatives, as noise can amplify errors.  

### 3. **Integral Control for Eliminating Steady-State Errors**  
   While the PD controller ensures good transient response, it can leave steady-state errors in scenarios with persistent disturbances (e.g., unmodeled drag). Incorporating an integral term to create a PID controller would:  
   - Accumulate errors over time and adjust control inputs to achieve zero steady-state error.  
   - Improve accuracy when the system operates near its thrust limits.  

### 4. **Trajectory Optimization for Smooth Motion**  
   The current system is tested primarily with step inputs or fixed setpoints. Extending the system to handle optimized trajectories could improve performance by:  
   - Designing time-optimal or smooth trajectories (e.g., splines or polynomials) for transitions between positions.  
   - Ensuring control actions respect the thrust constraints $u_{\text{min}}$ and $u_{\text{max}}\$ while minimizing oscillations and abrupt accelerations.  
   - Balancing response time and precision for practical applications, such as delivery systems or height-following tasks.  

### 5. **Nonlinear Control Strategies**  
   Although the current PD controller works well for linear approximations, the quadrotor dynamics are inherently nonlinear. Nonlinear control methods can provide enhanced performance:  
   - **Feedback Linearization**: Simplify nonlinear dynamics for controller design while ensuring better control fidelity.  
   - **Sliding Mode Control**: Enhance robustness under parameter uncertainties and disturbances.  
   - **Lyapunov-Based Control**: Offer formal stability guarantees while addressing the nonlinear nature of the quadrotor's dynamics.  

### 6. **Model Refinement and Parameter Adaptation**  
   The current model assumes ideal conditions with constant parameters. Enhancing the model can improve control performance:  
   - **Incorporate Additional Dynamics**: Account for unmodeled effects like drag forces or actuator response time.  
   - **Parameter Estimation**: Implement online parameter estimation algorithms to detect and adapt to changes in mass (e.g., varying payloads) or thrust coefficients.  

### 7. **Evaluation in Realistic Scenarios**  
   The system can be further validated by simulating or implementing more realistic conditions:  
   - **Random Disturbances**: Introduce stochastic disturbances, such as fluctuating wind forces, to test robustness.  
   - **Sensor Noise and Latency**: Simulate sensor inaccuracies and delays to evaluate the impact on control performance.  
   - **Extreme Conditions**: Test the system under challenging initial conditions (e.g., large deviations from equilibrium) to evaluate stability margins.  

### 8. **Learning-Based Control Enhancements**  
   Machine learning techniques can further optimize the control system:  
   - **Reinforcement Learning**: Use simulation to train an agent to derive optimal control policies that account for nonlinearities and disturbances.  
   - **Neural Networks**: Approximate the nonlinear dynamics and improve the controller’s ability to respond to complex conditions.  
   - **Supervised Learning**: Use historical data to fine-tune gains or predict required thrust for specific scenarios.  

By focusing on these areas, the 2D quadrotor control system can become more adaptable, precise, and robust for real-world applications without the need to expand into higher dimensions.

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgments

Inspired by the course Aerial Robotics offered by the University of Pennsylvania on Coursera.

## References

- MATLAB Documentation:
   - [ODE45](https://www.mathworks.com/help/matlab/ref/ode45.html)
   - [Graphics](https://www.mathworks.com/help/matlab/graphics.html)
   - [OpenGL](https://www.mathworks.com/help/matlab/ref/opengl.html)
- Coursera Course [Aerial Robotics by University of Pennsylvania](https://www.coursera.org/learn/robotics-flight)
- Control Theory Concepts: K. Ogata, Modern Control Engineering, Prentice Hall
