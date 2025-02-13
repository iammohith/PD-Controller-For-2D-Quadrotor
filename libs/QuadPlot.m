classdef QuadPlot < handle
    %QUADPLOT Visualization class for a quadrotor.
    %   This class manages the plotting of a quadrotor’s state in a 3D scene,
    %   including motor positions, history of states, and display of the quad's number.
    
    properties (SetAccess = public)
        k = 0;              % Iteration counter (for history updates)
        qn;                 % Quadrotor identifier (quad number)
        time = 0;           % Current time
        state;              % State vector (position, velocity, etc.)
        rot;                % Rotation matrix (body-to-world)
        
        color;              % Plot color for the quadrotor
        wingspan;           % Wingspan of the quadrotor (used for motor placement)
        height;             % Height offset of the quadrotor (for visualization)
        motor;              % Motor positions in the world frame
        
        state_hist;         % History of state vectors (for plotting trajectories)
        time_hist;          % History of time stamps
        max_iter;           % Maximum number of iterations (size of history arrays)
    end

    properties (SetAccess = private)
        h_3d;               % Handle for the 3D plot axes
        h_m13;              % Handle for the plot of motors 1 and 3
        h_m24;              % Handle for the plot of motors 2 and 4
        h_qz;               % Handle for the quadrotor's z-axis plot
        h_qn;               % Handle for the quad number text display
        h_pos_hist;         % Handle for the position history plot
        text_dist;          % Offset distance for displaying the quad number text
    end

    methods
        % Constructor: Initializes the QuadPlot object.
        function Q = QuadPlot(qn, state, wingspan, height, color, max_iter, h_3d)
            Q.qn = qn;
            Q.state = state;
            Q.wingspan = wingspan;
            Q.color = color;
            Q.height = height;
            
            % Convert quaternion (elements 7 to 10 of state) to rotation matrix
            Q.rot = QuatToRot(Q.state(7:10));
            % Calculate motor positions in world frame using the quad_pos function
            Q.motor = quad_pos(Q.state(1:3), Q.rot, Q.wingspan, Q.height);
            % Set the text offset for quad number display relative to wingspan
            Q.text_dist = Q.wingspan / 3;

            Q.max_iter = max_iter;
            % Preallocate history arrays for efficiency
            Q.state_hist = zeros(6, max_iter);
            Q.time_hist = zeros(1, max_iter);

            % If the plot handle is not provided, use current axes (gca)
            if nargin < 7
                h_3d = gca;
            end
            Q.h_3d = h_3d;
            
            % Prepare the plot by holding current content
            hold(Q.h_3d, 'on');
            
            % Plot initial position history (as a red dot)
            Q.h_pos_hist = plot3(Q.h_3d, Q.state(1), Q.state(2), Q.state(3), 'r.');
            
            % Plot motors 1 and 3 using indices 1 and 3 from the motor positions
            Q.h_m13 = plot3(Q.h_3d, ...
                Q.motor(1, [1 3]), ...
                Q.motor(2, [1 3]), ...
                Q.motor(3, [1 3]), ...
                '-ko', 'MarkerFaceColor', Q.color, 'MarkerSize', 5);
            
            % Plot motors 2 and 4 using indices 2 and 4 from the motor positions
            Q.h_m24 = plot3(Q.h_3d, ...
                Q.motor(1, [2 4]), ...
                Q.motor(2, [2 4]), ...
                Q.motor(3, [2 4]), ...
                '-ko', 'MarkerFaceColor', Q.color, 'MarkerSize', 5);
            
            % Plot the quadrotor's z-axis (using indices 5 and 6 from the motor positions)
            Q.h_qz = plot3(Q.h_3d, ...
                Q.motor(1, [5 6]), ...
                Q.motor(2, [5 6]), ...
                Q.motor(3, [5 6]), ...
                'Color', Q.color, 'LineWidth', 2);
            
            % Display the quad number as text near the quadrotor's position
            Q.h_qn = text(...
                Q.motor(1, 5) + Q.text_dist, ...
                Q.motor(2, 5) + Q.text_dist, ...
                Q.motor(3, 5) + Q.text_dist, num2str(qn));
            
            % Release the hold on the current plot
            hold(Q.h_3d, 'off');
        end

        % UpdateQuadState: Updates the state and rotation matrix based on new state data.
        function UpdateQuadState(Q, state, time)
            Q.state = state;
            Q.time = time;
            % Update the rotation matrix (transpose to convert to body-to-world)
            Q.rot = QuatToRot(state(7:10))';
        end

        % UpdateQuadHist: Records the current state and time into the history arrays.
        function UpdateQuadHist(Q)
            Q.k = Q.k + 1;
            Q.time_hist(Q.k) = Q.time;
            Q.state_hist(:, Q.k) = Q.state(1:6);
        end

        % UpdateMotorPos: Recalculates motor positions in the world frame.
        function UpdateMotorPos(Q)
            Q.motor = quad_pos(Q.state(1:3), Q.rot, Q.wingspan, Q.height);
        end

        % TruncateHist: Truncates the history arrays to the current iteration count.
        function TruncateHist(Q)
            Q.time_hist = Q.time_hist(1:Q.k);
            Q.state_hist = Q.state_hist(:, 1:Q.k);
        end

        % UpdateQuadPlot: Refreshes the visualization with the updated state.
        function UpdateQuadPlot(Q, state, time)
            % Update state, history, and motor positions
            Q.UpdateQuadState(state, time);
            Q.UpdateQuadHist();
            Q.UpdateMotorPos();
            
            % Update plot for motors 1 and 3
            set(Q.h_m13, ...
                'XData', Q.motor(1, [1 3]), ...
                'YData', Q.motor(2, [1 3]), ...
                'ZData', Q.motor(3, [1 3]));
            
            % Update plot for motors 2 and 4
            set(Q.h_m24, ...
                'XData', Q.motor(1, [2 4]), ...
                'YData', Q.motor(2, [2 4]), ...
                'ZData', Q.motor(3, [2 4]));
            
            % Update the quadrotor's z-axis plot
            set(Q.h_qz, ...
                'XData', Q.motor(1, [5 6]), ...
                'YData', Q.motor(2, [5 6]), ...
                'ZData', Q.motor(3, [5 6]));
            
            % Update the position of the quad number text display
            set(Q.h_qn, 'Position', ...
                [Q.motor(1, 5) + Q.text_dist, ...
                 Q.motor(2, 5) + Q.text_dist, ...
                 Q.motor(3, 5) + Q.text_dist]);
            
            % Update the historical trajectory plot
            set(Q.h_pos_hist, ...
                'XData', Q.state_hist(1, 1:Q.k), ...
                'YData', Q.state_hist(2, 1:Q.k), ...
                'ZData', Q.state_hist(3, 1:Q.k));
            
            % Force an immediate update of the figure
            drawnow;
        end
    end
end