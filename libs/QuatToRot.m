function R = QuatToRot(q)
    % QuatToRot Converts a quaternion to a 3x3 rotation matrix.
    %
    %   R = QuatToRot(q) returns the rotation matrix corresponding to the 
    %   input quaternion q. The quaternion is assumed to be in the form 
    %   [q0; q1; q2; q3], where q0 is the scalar part and q1:q3 is the 
    %   vector part.
    %
    %   Written by Daniel Mellinger.
    
    % Normalize the quaternion to ensure it represents a valid rotation.
    q = q ./ sqrt(sum(q.^2));
    
    % Construct the skew-symmetric matrix from the vector part of q.
    % This matrix is used to compute the rotation matrix.
    qahat = zeros(3,3); 
    qahat(1,2) = -q(4);
    qahat(1,3) =  q(3);
    qahat(2,1) =  q(4);
    qahat(2,3) = -q(2);
    qahat(3,1) = -q(3);
    qahat(3,2) =  q(2);
    
    % Compute the rotation matrix using the formula:
    % R = I + 2*qahat^2 + 2*q0*qahat
    R = eye(3) + 2*(qahat * qahat) + 2*q(1)*qahat;
end