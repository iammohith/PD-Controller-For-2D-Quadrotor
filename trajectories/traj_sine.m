function desired_state = traj_sine(t, ~)
    % traj_sine Generates the desired state for a combined sine trajectory.
    %
    %   desired_state = traj_sine(t, ~) computes the desired position, velocity,
    %   and acceleration for a trajectory where the y-coordinate follows a 
    %   piecewise linear motion (acceleration, constant speed, deceleration) and 
    %   the z-coordinate follows a sinusoidal profile.
    %
    %   Inputs:
    %       t - Current time.
    %       ~ - Unused parameter.
    %
    %   Output:
    %       desired_state - Structure with fields:
    %                           pos: 2x1 desired position [y; z]
    %                           vel: 2x1 desired velocity [y_dot; z_dot]
    %                           acc: 2x1 desired acceleration [y_ddot; z_ddot]
    
    % Initial position offset (starting at the origin).
    initial_pos = [0; 0];
    
    % Maximum velocity and acceleration for the y-motion.
    v_max_y = 2;
    a_max_y = 2;
    
    %% Y-coordinate motion (piecewise linear: acceleration, constant, deceleration)
    if t <= v_max_y / a_max_y
        % Acceleration phase.
        dt = t;
        acc_y = a_max_y;
        vel_y = acc_y * dt;
        pos_y = 0.5 * acc_y * dt^2;
    elseif t <= 2 * v_max_y / a_max_y
        % Constant velocity phase.
        dt = t - v_max_y / a_max_y;
        acc_y = 0;
        vel_y = v_max_y;
        pos_y = v_max_y^2 / (2 * a_max_y) + v_max_y * dt;
    elseif t <= 3 * v_max_y / a_max_y
        % Deceleration phase.
        dt = t - 2 * v_max_y / a_max_y;
        acc_y = -a_max_y;
        vel_y = v_max_y + acc_y * dt;
        pos_y = 3 * v_max_y^2 / (2 * a_max_y) + v_max_y * dt + 0.5 * acc_y * dt^2;
    else
        % After the motion is complete, hold the final position.
        acc_y = 0;
        vel_y = 0;
        pos_y = 2 * v_max_y^2 / a_max_y;
    end
    
    %% Z-coordinate motion (sinusoidal profile)
    % Define the total duration for the y-motion (end of deceleration phase).
    t_max = 3 * v_max_y / a_max_y;
    % Define angular frequency such that the sine completes a prescribed oscillation.
    omega = 4 * pi / t_max;
    
    if t < t_max
        % Compute sinusoidal position, velocity, and acceleration for z.
        pos_z = 0.25 * (1 - cos(omega * t));
        vel_z = 0.25 * omega * sin(omega * t);
        acc_z = 0.25 * (omega^2) * cos(omega * t);
    else
        % After t_max, hold the final sine position.
        pos_z = 0.25 * (1 - cos(omega * t_max));
        vel_z = 0;
        acc_z = 0;
    end
    
    %% Combine the y and z components with the initial offset.
    desired_state.pos = initial_pos + [pos_y; pos_z];
    desired_state.vel = [vel_y; vel_z];
    desired_state.acc = [acc_y; acc_z];
end