function [t_out, s_out] = simulation_2d(controlhandle, trajhandle)
    % simulation_2d Simulates a 2D system using ODE45 and displays the result in real-time.
    %
    % Inputs:
    %   controlhandle - Function handle for the control system.
    %   trajhandle    - Function handle for the trajectory generator.
    %
    % Outputs:
    %   t_out - Time vector of the simulation.
    %   s_out - State trajectory from the simulation.

    % Retrieve system parameters.
    params = sys_params;

    % Flag to run simulation in real-time (pauses to sync with simulation time).
    real_time = true;

    %% **************************** FIGURE SETUP *****************************
    disp('Initializing figures...')

    % Create a figure and center it on the screen.
    h_fig = figure;
    sz = [790 607];               % Figure size [width, height].
    screensize = get(0, 'ScreenSize');
    xpos = ceil((screensize(3) - sz(1)) / 2); % Center horizontally.
    ypos = ceil((screensize(4) - sz(2)) / 2); % Center vertically.
    set(h_fig, 'Position', [xpos ypos sz]);

    % Create a subplot for the main 2D (quad) plot.
    h_3d = subplot(3, 3, [1,2,4,5,7,8]);
    axis equal;
    grid on;
    view(90, 0);
    ylabel('y [m]');
    zlabel('z [m]');

    quadcolors = lines(1);   % Set the color scheme for the quad plot.

    % Set renderer to OpenGL for improved graphics performance.
    set(gcf, 'Renderer', 'OpenGL');

    %% *********************** INITIAL CONDITIONS ***********************
    % Set total simulation time based on the trajectory function.
    if isequal(trajhandle, @traj_diamond)
        t_total = 13;
    else
        t_total = 5;
    end

    tstep    = 0.01;             % Time step for ODE solver.
    cstep    = 0.05;             % Time interval for plotting updates.
    max_iter = t_total / cstep;   % Maximum number of iterations.
    nstep    = cstep / tstep;     % Number of simulation steps per iteration.
    time     = 0;                % Initialize current simulation time.
    err = [];                    % Variable to store runtime error messages.

    % Retrieve start and stop positions from the trajectory function.
    des_start = trajhandle(0, []);
    des_stop  = trajhandle(inf, []);

    % Determine plot boundaries based on the desired trajectory.
    d_state = nan(max_iter, 2);
    for iter = 1:max_iter
        dd = trajhandle(cstep * iter, []);
        d_state(iter, :) = dd.pos(1:2)';
    end
    y_lim = [min(d_state(:, 1)) - 0.1, max(d_state(:, 1)) + 0.1];
    z_lim = [min(d_state(:, 2)) - 0.1, max(d_state(:, 2)) + 0.1];
    if (4 * (z_lim(2) - z_lim(1)) < y_lim(2) - y_lim(1))
        z_lim(1) = z_lim(1) - (y_lim(2) - y_lim(1)) / 8;
        z_lim(2) = z_lim(2) + (y_lim(2) - y_lim(1)) / 8;
    end

    stop_pos = des_stop.pos;  % Final desired position.

    % Initial state: [position; angle; velocity; angular velocity].
    x0 = [des_start.pos; 0; des_start.vel; 0];

    % Preallocate arrays for state and time trajectories.
    xtraj = nan(max_iter * nstep, length(x0));
    ttraj = nan(max_iter * nstep, 1);

    x = x0;  % Set initial state.

    % Tolerances for termination criteria.
    pos_tol = 0.01;
    vel_tol = 0.03;
    ang_tol = 0.05;

    %% ************************* RUN SIMULATION *************************
    disp('Simulation Running....')

    % Main simulation loop.
    for iter = 1:max_iter
        % Define the time interval for the current iteration.
        timeint = time : tstep : time + cstep;

        tic;  % Start timing this iteration.

        % Initialize quad plot on the first iteration.
        if iter == 1
            subplot(3, 3, [1,2,4,5,7,8]);
            quad_state = simStateToQuadState(x0);
            QP = QuadPlot(1, quad_state, params.arm_length, 0.05, quadcolors(1, :), max_iter, h_3d);
            ylim(y_lim); 
            zlim(z_lim);
            quad_state = simStateToQuadState(x);
            QP.UpdateQuadPlot(quad_state, time);
            h_title = title(h_3d, sprintf('iteration: %d, time: %4.2f', iter, time));
        end

        % Run simulation for the current time interval using ode45.
        [tsave, xsave] = ode45(@(t, s) sys_eom(t, s, controlhandle, trajhandle, params), timeint, x);
        x = xsave(end, :)';  % Update the state to the last computed state.

        % Save simulation results for trajectory plotting.
        xtraj((iter - 1) * nstep + 1 : iter * nstep, :) = xsave(1:end-1, :);
        ttraj((iter - 1) * nstep + 1 : iter * nstep) = tsave(1:end-1);

        % Update the quad plot with the new state.
        quad_state = simStateToQuadState(x);
        QP.UpdateQuadPlot(quad_state, time + cstep);
        subplot(3, 3, [1,2,4,5,7,8]);
        ylim(y_lim); 
        zlim(z_lim);
        set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep));
        time = time + cstep;  % Update simulation time.

        % Update subplot for y-position vs. time.
        subplot(3, 3, 3);
        plot(ttraj(1:iter * nstep), xtraj(1:iter * nstep, 1));
        xlabel('t [s]');
        ylabel('y [m]');
        grid on;

        % Update subplot for z-position vs. time.
        subplot(3, 3, 6);
        plot(ttraj(1:iter * nstep), xtraj(1:iter * nstep, 2));
        xlabel('t [s]');
        ylabel('z [m]');
        grid on;

        % Update subplot for phi (angle in degrees) vs. time.
        subplot(3, 3, 9);
        plot(ttraj(1:iter * nstep), 180/pi * xtraj(1:iter * nstep, 3));
        xlabel('t [s]');
        ylabel('\phi [∠]');
        grid on;

        t_elapsed = toc;  % Stop timing this iteration.

        % Check if the ODE solver is taking too long (potential instability).
        if t_elapsed > cstep * 500
            err = 'Ode45 Unstable';
            break;
        end

        % Pause to sync with real-time if enabled.
        if real_time && (t_elapsed < cstep)
            pause(cstep - t_elapsed);
        end

        % Check termination criteria based on position, velocity, and angle tolerances.
        if time > t_total - 0.001 
            if norm(x(1:2) - stop_pos) < pos_tol && norm(x(4:5)) < vel_tol && abs(x(3)) < ang_tol
                err = [];
                break;
            elseif norm(x(1:2) - stop_pos) > pos_tol
                err = 'Did not reach goal';
            elseif norm(x(4:5)) > vel_tol
                err = 'Velocity not close to zero';
            elseif abs(x(3)) > ang_tol
                err = 'Final angle not close to zero';
            end
        end
    end

    disp('Simulation done');

    % Return results if simulation completed without error.
    if ~isempty(err)
        disp(['Error: ', err]);
        t_out = [];
        s_out = [];
    else
        disp(['Final time: ', num2str(time), ' sec']);
        t_out = ttraj(1:iter * nstep);
        s_out = xtraj(1:iter * nstep, :);
    end
end