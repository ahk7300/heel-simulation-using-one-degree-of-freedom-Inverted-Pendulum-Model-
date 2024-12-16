% Inverted Pendulum Simulation with and without Heel Support
clear; clc; close all;  % Clear workspace, clear command window, and close all figures

% --- Fixed Parameters ---
mass = 1.0;                   % Mass of the body segment (kg)
segment_length = 0.5;         % Length of the body segment (m)
g = 9.81;                     % Acceleration due to gravity (m/s^2)
theta0 = deg2rad(10);         % Initial angle (converted from 10 degrees to radians)
omega0 = 0;                   % Initial angular velocity (rad/s)
T_end = 10;                   % Duration of the simulation (10 seconds)
heel_effect_factor = 0.9;     % Heel support reduces effective length by 10%
fps = 60;                     % Frames per second for the animation

% Effective lengths for with and without heel support
effective_lengths = [heel_effect_factor * segment_length, segment_length]; % [With Heel, Without Heel]

% Initial conditions for the simulation (initial angle and angular velocity)
initial_conditions = [theta0; omega0];

% Time span for the ode45 solver (from 0 to T_end)
time_span = [0, T_end];

% --- Solve ODEs ---
solutions = cell(2, 1);  % Cell array to store solutions for both conditions (with and without heel)
times = cell(2, 1);      % Cell array to store time vectors for both conditions

% Solve the ODE for both cases (with and without heel)
for i = 1:2
    % Define the ODE function for the pendulum with respect to its angular position and velocity
    pendulum_ode = @(t, y) [y(2); -(g / effective_lengths(i)) * sin(y(1))];
    
    % Solve the ODE using ode45, which returns the time vector and solution matrix (theta, omega)
    [times{i}, solutions{i}] = ode45(pendulum_ode, time_span, initial_conditions);
end

% Extract angles (theta) and angular velocities (omega) from the solution matrix
theta_heel = solutions{1}(:, 1);  % Angle with heel support
omega_heel = solutions{1}(:, 2);  % Angular velocity with heel support
theta_no_heel = solutions{2}(:, 1);  % Angle without heel support
omega_no_heel = solutions{2}(:, 2);  % Angular velocity without heel support

% --- Plot Results ---
figure;  % Create a new figure for plotting

% Plot angle (in degrees) over time
subplot(2, 1, 1);  % Create the first subplot (upper plot)
plot(times{1}, rad2deg(theta_heel), 'b', 'LineWidth', 1.5); hold on;  % Plot the angle with heel support in blue
plot(times{2}, rad2deg(theta_no_heel), 'r', 'LineWidth', 1.5);  % Plot the angle without heel support in red
title('Inverted Pendulum - Angle Over Time');  % Title of the plot
xlabel('Time (s)');  % X-axis label
ylabel('Angle (degrees)');  % Y-axis label
legend('With Heel Support', 'Without Heel Support');  % Legend for the plot
grid on;  % Turn on the grid

% Plot angular velocity (in rad/s) over time
subplot(2, 1, 2);  % Create the second subplot (lower plot)
plot(times{1}, omega_heel, 'b', 'LineWidth', 1.5); hold on;  % Plot angular velocity with heel support in blue
plot(times{2}, omega_no_heel, 'r', 'LineWidth', 1.5);  % Plot angular velocity without heel support in red
title('Inverted Pendulum - Angular Velocity Over Time');  % Title of the plot
xlabel('Time (s)');  % X-axis label
ylabel('Angular Velocity (rad/s)');  % Y-axis label
legend('With Heel Support', 'Without Heel Support');  % Legend for the plot
grid on;  % Turn on the grid

disp('ODE45 Simulation completed.');  % Display a message when the ODE simulation is done

% --- Real-Time Animation ---
figure;  % Create a new figure for animation
animation_length = 1.2 * segment_length;  % Set the axis limit for animation (slightly larger than segment length)

colors = ['b', 'r'];  % Define colors for the two cases (with heel in blue, without heel in red)

% Set up subplots for animation (one subplot for each condition)
for i = 1:2
    subplot(1, 2, i);  % Create two subplots in a single row (1 row, 2 columns)
    hold on;  % Hold the plot to update it in each frame
    if i == 1
        title('With Heel Support');  % Set title for the first subplot
    else
        title('Without Heel Support');  % Set title for the second subplot
    end
    xlabel('X Position (m)');  % X-axis label
    ylabel('Y Position (m)');  % Y-axis label
    axis equal;  % Set equal scaling for both axes
    axis([-animation_length animation_length -animation_length animation_length]);  % Set axis limits
    grid on;  % Turn on the grid
end

% Precompute interpolated solutions for animation (to match the frame rate)
t_anim = 0:(1 / fps):T_end;  % Time vector for animation (with intervals based on FPS)
theta_interp = cell(2, 1);  % Cell array to store interpolated angles for both cases

% Interpolate the angle solutions for both cases (heel support and no heel)
for i = 1:2
    theta_interp{i} = interp1(times{i}, solutions{i}(:, 1), t_anim, 'linear', 'extrap');  % Linear interpolation with extrapolation
end

% Initialize animation objects (lines and markers for each pendulum)
lines = gobjects(2, 1);  % Initialize graphics objects for the lines (pendulum rods)
markers = gobjects(2, 1);  % Initialize graphics objects for the markers (pendulum bob)

% Set up the line and marker for both subplots (with and without heel support)
for i = 1:2
    subplot(1, 2, i);  % Select the correct subplot
    lines(i) = line([0, 0], [0, 0], 'Color', colors(i), 'LineWidth', 2);  % Create line for pendulum rod
    markers(i) = plot(0, 0, 'o', 'Color', colors(i), 'MarkerSize', 10, 'MarkerFaceColor', colors(i));  % Create marker for pendulum bob
end

% Animation loop
disp('Starting real-time animation...');  % Display message when animation starts
for i = 1:length(t_anim)  % Loop over each time step in the animation
    for j = 1:2  % Loop over the two cases (heel support and no heel support)
        % Calculate the x and y positions of the pendulum bob
        x = effective_lengths(j) * sin(theta_interp{j}(i));  % X position
        y = -effective_lengths(j) * cos(theta_interp{j}(i));  % Y position
        
        % Update the line (pendulum rod) and marker (bob) positions
        set(lines(j), 'XData', [0, x], 'YData', [0, y]);  % Update the rod line
        set(markers(j), 'XData', x, 'YData', y);  % Update the bob position
    end
    pause(1 / fps);  % Pause for the frame rate duration
end

disp('Real-time animation completed.');  % Display message when animation is done
