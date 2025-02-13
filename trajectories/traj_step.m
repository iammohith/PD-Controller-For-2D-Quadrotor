function desired_state = traj_step(t, ~)
    % traj_step Generates the desired state for a step input trajectory.
    %
    %   desired_state = traj_step(t, ~) computes the desired position, velocity,
    %   and acceleration for a step input. The output position remains at [0; 0]
    %   for t < 1, and switches to [0.5; 0.0] for t >= 1. The velocity and
    %   acceleration are zero throughout.
    %
    %   Inputs:
    %       t - Current time.
    %       ~ - Unused parameter.
    %
    %   Output:
    %       desired_state - Structure with fields:
    %                           pos: 2x1 desired position.
    %                           vel: 2x1 desired velocity (zero vector).
    %                           acc: 2x1 desired acceleration (zero vector).

    % Initialize acceleration and velocity as zero vectors.
    acc = zeros(2, 1);
    vel = zeros(2, 1);
    
    % Set position based on the current time.
    if t < 1
        pos = zeros(2, 1);
    else
        pos = [0.5; 0.0];
    end

    % Assemble the desired state structure.
    desired_state.pos = pos;
    desired_state.vel = vel;
    desired_state.acc = acc;
end