function [params] = sys_params()
    % sys_params Returns system parameters for the 2D quadrotor model.
    %
    %   params = sys_params() returns a structure containing the physical
    %   parameters of the quadrotor system, including gravity, mass, moment
    %   of inertia, arm length, and motor force limits.
    %
    %   The parameters include:
    %       gravity    - Gravitational acceleration (m/s^2)
    %       mass       - Mass of the quadrotor (kg)
    %       Ixx        - Moment of inertia about the x-axis (kg*m^2)
    %       arm_length - Distance from the center to each motor (m)
    %       minF       - Minimum allowable force per motor (N)
    %       maxF       - Maximum allowable total force from both motors (N)
    
    % Define physical constants and quadrotor properties.
    params.gravity = 9.81;                 % Gravitational acceleration (m/s^2)
    params.mass = 0.18;                    % Mass of the quadrotor (kg)
    params.Ixx = 0.00025;                  % Moment of inertia about the x-axis (kg*m^2)
    params.arm_length = 0.086;             % Arm length (m)
    
    % Define force limits for the motors.
    params.minF = 0;                     % Minimum motor force (N)
    params.maxF = 2 * params.mass * params.gravity; % Maximum total force (N)
end