function [quad] = quad_pos(pos, rot, L, H)
    % QUAD_POS Calculates the coordinates of a quadrotor's key points in the world frame.
    %
    %   quad = quad_pos(pos, rot, L, H) transforms selected body-frame points of the quadrotor 
    %   into the world frame using the provided position and rotation.
    %
    % Inputs:
    %   pos - 3x1 position vector [x; y; z] representing the quadrotor's location.
    %   rot - 3x3 body-to-world rotation matrix.
    %   L   - Scalar representing the lateral length offset from the center.
    %   H   - (Optional) Height offset; defaults to 0.05 if not provided.
    %
    % Output:
    %   quad - 3xN matrix containing the transformed coordinates of the quadrotor's points.
    
    % Set default height offset if not provided
    if nargin < 4
        H = 0.05;
    end

    % Construct the homogeneous transformation matrix from the body frame to the world frame.
    % This matrix incorporates the rotation and translation (position).
    wHb = [rot, pos(:); 0, 0, 0, 1];

    % Define key points in the quadrotor body frame (in homogeneous coordinates).
    % Points: [L; 0; 0], [0; L; 0], [-L; 0; 0], [0; -L; 0], [0; 0; 0], [0; 0; H].
    quadBodyFrame = [ L,  0,  0, 1;
                      0,  L,  0, 1;
                     -L,  0,  0, 1;
                      0, -L,  0, 1;
                      0,  0,  0, 1;
                      0,  0,  H, 1]';

    % Transform the body-frame points into the world frame.
    quadWorldFrame = wHb * quadBodyFrame;
    
    % Extract the x, y, z coordinates (remove the homogeneous coordinate).
    quad = quadWorldFrame(1:3, :);
end