function quad_state = simStateToQuadState(sim_state)
    % simStateToQuadState Convert simulation state to a 13-element quadrotor state.
    %
    %   This function converts a 6-element simulation state vector to a 13-element
    %   quadrotor state vector.
    %
    %   Input:
    %       sim_state: 6x1 vector [y; z; phi; y_dot; z_dot; phi_dot]
    %
    %   Output:
    %       quad_state: 13x1 vector [x; y; z; xdot; ydot; zdot; qw; qx; qy; qz; p; q; r]
    %
    %   Mapping:
    %       - Position: x is set to 0, y and z are taken from sim_state.
    %       - Linear velocities: xdot is set to 0; ydot and zdot are from sim_state.
    %       - Orientation: A quaternion is constructed from the angle phi
    %         (with q = [cos(-phi/2); sin(-phi/2); 0; 0]).
    %       - Angular velocities: p and r are set to 0; q is taken from sim_state.
    
    % Preallocate the quadrotor state vector with zeros
    quad_state = zeros(13, 1);
    
    % Position (x, y, z)
    quad_state(1) = 0;              % x-position (assumed 0)
    quad_state(2) = sim_state(1);     % y-position from simulation state
    quad_state(3) = sim_state(2);     % z-position from simulation state
    
    % Linear velocities (xdot, ydot, zdot)
    quad_state(4) = 0;              % x-velocity (assumed 0)
    quad_state(5) = sim_state(4);     % y-velocity from simulation state
    quad_state(6) = sim_state(5);     % z-velocity from simulation state
    
    % Orientation (quaternion: qw, qx, qy, qz)
    quad_state(7)  = cos(-sim_state(3)/2);  % qw
    quad_state(8)  = sin(-sim_state(3)/2);  % qx
    quad_state(9)  = 0;                     % qy (assumed 0)
    quad_state(10) = 0;                     % qz (assumed 0)
    
    % Angular velocities (p, q, r)
    quad_state(11) = 0;              % p (assumed 0)
    quad_state(12) = sim_state(6);     % q from simulation state
    quad_state(13) = 0;              % r (assumed 0)
end