clear;
close all;

% Add required directories containing utility and trajectory functions
addpath('libs');
addpath('trajectories');

% Define function handles for the controller and trajectory generator
controlhandle = @controller;
trajhandle = @traj_step;

% Run the 2D simulation using the specified control and trajectory functions
[t, state] = simulation_2d(controlhandle, trajhandle);