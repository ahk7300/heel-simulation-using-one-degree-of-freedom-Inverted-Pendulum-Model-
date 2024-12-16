% Inverted Pendulum Simulation around the Ankle - With and Without Heel Support using ode45
% This code simulates an inverted pendulum with fixed parameters for two conditions:
% with and without heel support, using the ODE solver ode45.

% Clear workspace and close figures
clear;
clc;
close all;

% Fixed parameters for the simulation
mass = 1.0;                   % Mass of the body segment (kg)
segment_length = 0.5;         % Length of the body segment from ankle to knee (m)
g = 9.81;                     % Acceleration due to gravity (m/s^2)
theta0 = deg2rad(10);         % Initial angle (10 degrees), converted to radians
omega0 = 0;                   % Initial angular velocity (rad/s)
T_end = 10;                   % End time for simulation (s)
heel_effect_factor = 0.9;     % Effective length reduced by 10% for heel support

% Effective lengths with and without heel support
effective_length_heel = heel_effect_factor * segment_length;
effective_length_no_heel = segment_length;

% Initial conditions vector [initial angle, initial angular velocity]
initial_conditions = [theta0; omega0];

% Time span for ode45
time_span = [0, T_end];

% Define the differential equations as functions

% 1. With heel support
pendulum_with_heel = @(t, y) [y(2); -(g / effective_length_heel) * sin(y(1))];

% 2. Without heel support
pendulum_without_heel = @(t, y) [y(2); -(g / effective_length_no_heel) * sin(y(1))];

% Solve ODEs
[time_heel, solution_heel] = ode45(pendulum_with_heel, time_span, initial_conditions);
[time_no_heel, solution_no_heel] = ode45(pendulum_without_heel, time_span, initial_conditions);

% Extract angles and angular velocities
theta_heel = solution_heel(:, 1);        % Angle with heel support
omega_heel = solution_heel(:, 2);        % Angular velocity with heel support
theta_no_heel = solution_no_heel(:, 1);  % Angle without heel support
omega_no_heel = solution_no_heel(:, 2);  % Angular velocity without heel support

% Plot results
figure;

% Plot angle over time
subplot(2, 1, 1);
plot(time_heel, rad2deg(theta_heel), 'b', 'LineWidth', 1.5);
hold on;
plot(time_no_heel, rad2deg(theta_no_heel), 'r', 'LineWidth', 1.5);
title('Inverted Pendulum around Ankle using ODE45 - Angle Over Time');
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('With Heel Support', 'Without Heel Support');
grid on;

% Plot angular velocity over time
subplot(2, 1, 2);
plot(time_heel, omega_heel, 'b', 'LineWidth', 1.5);
hold on;
plot(time_no_heel, omega_no_heel, 'r', 'LineWidth', 1.5);
title('Inverted Pendulum around Ankle using ODE45 - Angular Velocity Over Time');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('With Heel Support', 'Without Heel Support');
grid on;

disp('ODE45 Simulation completed.');
