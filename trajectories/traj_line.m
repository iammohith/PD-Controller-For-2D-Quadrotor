function desired_state = traj_line(t, ~)
    % traj_line Generates the desired state for a straight-line trajectory.
    %
    %   desired_state = traj_line(t, ~) computes the desired position, velocity,
    %   and acceleration along a line for a given time t.
    %
    %   Inputs:
    %       t - Current time.
    %       ~ - Unused parameter.
    %
    %   Output:
    %       desired_state - Structure containing:
    %                         pos: Desired position (2x1 vector)
    %                         vel: Desired velocity (2x1 vector)
    %                         acc: Desired acceleration (2x1 vector)
    
    % Initial position offset.
    initial_pos = [0; 1];
    
    % Define maximum velocity and acceleration.
    v_max = 2;
    a_max = 2;
    
    % The trajectory is divided into three phases:
    % 1. Acceleration phase: 0 <= t <= v_max/a_max.
    % 2. Constant velocity phase: v_max/a_max < t <= 2*v_max/a_max.
    % 3. Deceleration phase: 2*v_max/a_max < t <= 3*v_max/a_max.
    % Beyond these intervals, the motion stops.
    
    if t <= v_max/a_max
        % Acceleration phase.
        dt = t;
        acc = [a_max; 0];
        vel = acc * dt;
        pos = 0.5 * acc * dt^2;
    elseif t <= 2*v_max/a_max
        % Constant velocity phase.
        dt = t - v_max/a_max;
        acc = [0; 0];
        vel = [v_max; 0];
        pos = [v_max^2/(2*a_max); 0] + [v_max * dt; 0];
    elseif t <= 3*v_max/a_max
        % Deceleration phase.
        dt = t - 2*v_max/a_max;
        acc = [-a_max; 0];
        vel = [v_max; 0] + acc * dt;
        pos = [3*v_max^2/(2*a_max); 0] + [v_max; 0] * dt + 0.5 * acc * dt^2;
    else
        % After the trajectory is completed, hold final position.
        acc = [0; 0];
        vel = [0; 0];
        pos = [2*v_max^2/a_max; 0];
    end
    
    % Combine the initial position with the computed position offset.
    desired_state.pos = initial_pos + pos;
    desired_state.vel = vel;
    desired_state.acc = acc;
end