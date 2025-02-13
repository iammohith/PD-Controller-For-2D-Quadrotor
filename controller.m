function [u1, u2] = controller(~, state, des_state, params)
    % CONTROLLER  Controller for the planar quadrotor.
    %
    %   [u1, u2] = controller(~, state, des_state, params) computes the thrust
    %   (u1) and moment (u2) commands for a planar quadrotor based on the current
    %   and desired states.
    %
    %   Inputs:
    %       state: Structure containing the current state:
    %           state.pos   - Position [y; z]
    %           state.vel   - Velocity [y_dot; z_dot]
    %           state.rot   - Orientation angle (phi)
    %           state.omega - Angular velocity (phi_dot)
    %
    %       des_state: Structure containing the desired state:
    %           des_state.pos - Desired position [y; z]
    %           des_state.vel - Desired velocity [y_dot; z_dot]
    %           des_state.acc - Desired acceleration [y_ddot; z_ddot]
    %
    %       params: Structure containing robot parameters:
    %           params.gravity - Gravitational acceleration
    %           params.mass    - Mass of the quadrotor
    %           params.Ixx     - Moment of inertia about the x-axis
    %
    %   Outputs:
    %       u1: Total thrust force command.
    %       u2: Control moment (torque) command.
    %
    %   The controller uses PD control laws for both the vertical (z) dynamics and
    %   the lateral (y) dynamics. A desired pitch angle is computed to achieve the
    %   desired lateral acceleration, and then the thrust and moment are computed.
    
    % Control gains for vertical and lateral dynamics as well as for rotational control.
    kv_z = 20;
    kp_z = 200;
    kv_phi = 100;
    kp_phi = 1000;
    kv_y = 10;
    kp_y = 10;
    
    % Compute the error between desired and current positions and velocities.
    pos_error = des_state.pos - state.pos;
    vel_error = des_state.vel - state.vel;
    
    % Compute the commanded pitch angle (phi_commanded) to achieve desired lateral acceleration.
    % Note: The pitch angle is computed based on the desired acceleration in the y-direction.
    phi_commanded = -(1 / params.gravity) * (des_state.acc(1, 1) + kv_y * vel_error(1) + kp_y * pos_error(1));
    
    % Compute the error between the commanded and current pitch angles.
    phi_error = phi_commanded - state.rot;
    
    % Commanded angular velocity (phi_dot) is set to zero.
    phidot_commanded = 0;
    phidot_error = phidot_commanded - state.omega;
    
    % Assume the desired angular acceleration is zero.
    phi_double_dot = 0;
    
    % Compute the thrust command (u1) using vertical dynamics.
    % It compensates for gravity and tracks the desired vertical acceleration.
    u1 = params.mass * (params.gravity + des_state.acc(2, 1) + kv_z * vel_error(2) + kp_z * pos_error(2));
    
    % Compute the moment command (u2) using rotational dynamics.
    u2 = params.Ixx * (phi_double_dot + kv_phi * phidot_error + kp_phi * phi_error);
end