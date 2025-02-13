function [sdot] = sys_eom(t, s, controlhandle, trajhandle, params)
    % sys_eom Differential equation for the 2D quadrotor system.
    %
    %   sdot = sys_eom(t, s, controlhandle, trajhandle, params) computes the time
    %   derivative of the state vector s for a 2D quadrotor.
    %
    %   The state vector is defined as:
    %       s = [y; z; phi; y_dot; z_dot; phi_dot]
    %
    %   Inputs:
    %       t             - Current time.
    %       s             - Current state vector.
    %       controlhandle - Function handle to compute control inputs (F and M).
    %       trajhandle    - Function handle to generate the desired trajectory.
    %       params        - Structure containing system parameters (mass, gravity,
    %                       arm_length, Ixx, minF, maxF, etc.).
    %
    %   Output:
    %       sdot - Time derivative of the state vector.
    
    % Structure the current state for clarity.
    current_state.pos   = s(1:2);   % Position: [y; z]
    current_state.rot   = s(3);     % Orientation angle (phi)
    current_state.vel   = s(4:5);   % Linear velocities: [y_dot; z_dot]
    current_state.omega = s(6);     % Angular velocity: phi_dot
    
    % Compute the desired state using the provided trajectory function.
    desired_state = trajhandle(t, current_state);
    
    % Compute control inputs (total force F and moment M) from the control law.
    [F, M] = controlhandle(t, current_state, desired_state, params);
    
    % Compute individual motor forces.
    % Motor allocation:
    %      u1      u2
    %    _____    _____
    %      |________|
    % where:
    %   F = u1 + u2;
    %   M = (u2 - u1) * params.arm_length;
    u1 = 0.5 * (F - M/params.arm_length);
    u2 = 0.5 * (F + M/params.arm_length);
    
    % Clamp the motor forces to remain within specified limits.
    u1_clamped = min(max(params.minF/2, u1), params.maxF/2);
    u2_clamped = min(max(params.minF/2, u2), params.maxF/2);
    
    % Recalculate the clamped total force and moment.
    F_clamped = u1_clamped + u2_clamped;
    M_clamped = (u2_clamped - u1_clamped) * params.arm_length;
    
    % Compute the state derivative vector.
    sdot = [ s(4);                                % y_dot becomes derivative of y
             s(5);                                % z_dot becomes derivative of z
             s(6);                                % phi_dot becomes derivative of phi
            -F_clamped * sin(s(3)) / params.mass;   % Acceleration in y
             F_clamped * cos(s(3)) / params.mass - params.gravity;  % Acceleration in z
             M_clamped / params.Ixx];              % Angular acceleration (phi_ddot)
end