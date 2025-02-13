function desired_state = traj_diamond(t, cur_state)
    % traj_diamond Generates the desired state for a diamond-shaped trajectory.
    %
    %   desired_state = traj_diamond(t, cur_state) computes the desired position,
    %   velocity, and acceleration for a diamond trajectory based on the input time t.
    %
    %   Inputs:
    %       t         - Current time.
    %       cur_state - (Unused) current state structure.
    %
    %   Output:
    %       desired_state - Structure with fields:
    %                         pos: Desired position (2x1 vector)
    %                         vel: Desired velocity (2x1 vector)
    %                         acc: Desired acceleration (2x1 vector)
    
    % Define maximum velocity and acceleration
    v_max = 3;
    a_max = 4;
    
    % Compute initial positions for each segment of the diamond trajectory.
    % Each initial_pos corresponds to a vertex of the diamond.
    initial_pos1 = [0; 1.5];
    initial_pos2 = initial_pos1 + [cos(pi/4) -sin(pi/4); sin(pi/4) cos(pi/4)] * [2*v_max^2/a_max; 0];
    initial_pos3 = initial_pos2 + [cos(-pi/4) -sin(-pi/4); sin(-pi/4) cos(-pi/4)] * [2*v_max^2/a_max; 0];
    initial_pos4 = initial_pos3 + [cos(-3*pi/4) -sin(-3*pi/4); sin(-3*pi/4) cos(-3*pi/4)] * [2*v_max^2/a_max; 0];
    
    % Duration of one segment of the trajectory.
    t_seg = 4*v_max/a_max;
    
    % Clamp time to the total duration of 4 segments and compute the time within the current segment.
    if t >= 4*t_seg
        t = 4*t_seg;
        t_prune = t_seg;
    else
        t_prune = mod(max(0, t - 0.00001), t_seg);
    end
    
    % Determine the starting position for the current segment.
    if t <= t_seg
        initial_pos = initial_pos1;    
    elseif t <= 2*t_seg 
        initial_pos = initial_pos2;
    elseif t <= 3*t_seg 
        initial_pos = initial_pos3;
    else
        initial_pos = initial_pos4;
    end
    
    % Determine the rotation angle for the current segment.
    % The angle decreases by pi/2 for each completed segment.
    theta = pi/4 - floor(max(0, t - 0.00001)/t_seg) * (pi/2);
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    
    % Compute the position, velocity, and acceleration within the current segment.
    if t_prune <= v_max/a_max
        % Acceleration phase: increasing speed.
        dt = t_prune;
        acc = [a_max; 0];
        vel = acc * dt;
        pos = 0.5 * acc * dt^2;
    elseif t_prune <= 2*v_max/a_max
        % Constant velocity phase.
        dt = t_prune - v_max/a_max;
        acc = [0; 0];
        vel = [v_max; 0];
        pos = [v_max^2/(2*a_max); 0] + [v_max * dt; 0];
    elseif t_prune <= 3*v_max/a_max
        % Deceleration phase: reducing speed.
        dt = t_prune - 2*v_max/a_max;
        acc = [-a_max; 0];
        vel = [v_max; 0] + acc * dt;
        pos = [3*v_max^2/(2*a_max); 0] + [v_max; 0] * dt + 0.5 * acc * dt^2;
    else
        % End of segment: ensure final position is reached.
        acc = [0; 0];
        vel = [0; 0];
        pos = [2*v_max^2/a_max; 0];
    end
    
    % Rotate and translate the computed position, velocity, and acceleration
    % to align with the current segment's orientation.
    desired_state.pos = initial_pos + R * pos;
    desired_state.vel = R * vel;
    desired_state.acc = R * acc;
end