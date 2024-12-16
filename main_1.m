% Inverted Pendulum Simulation around the Ankle - With and Without Heel Support
% This code simulates an inverted pendulum with fixed parameters for two conditions:
% with and without heel support, pivoting around the ankle.

% Clear workspace and close figures
clear;
clc;
close all;

% Fixed parameters for the simulation
mass = 1.0;                  % Mass of the body segment (kg)
segment_length = 0.5;        % Length of the body segment from ankle to knee (m)
g = 9.81;                    % Acceleration due to gravity (m/s^2)
theta0 = deg2rad(10);        % Initial angle (10 degrees), converted to radians
dt = 0.01;                   % Time step (s)
T_end = 10;                  % End time for simulation (s)
time = 0:dt:T_end;           % Time vector

% Simulation with heel support (shorter effective length)
heel_effect_factor = 0.9;            % Effective length reduced by 10% for heel support
effective_length_heel = heel_effect_factor * segment_length;

% Simulation without heel support (full length)
effective_length_no_heel = segment_length;

% Preallocate arrays for results
theta_heel = zeros(1, numel(time));
omega_heel = zeros(1, numel(time));
theta_no_heel = zeros(1, numel(time));
omega_no_heel = zeros(1, numel(time));

% Initial conditions
theta_heel(1) = theta0;
theta_no_heel(1) = theta0;

% Simulation loop for both cases
for i = 1:numel(time) - 1
    % With heel support
    alpha_heel = -(g / effective_length_heel) * sin(theta_heel(i));
    omega_heel(i + 1) = omega_heel(i) + alpha_heel * dt;
    theta_heel(i + 1) = theta_heel(i) + omega_heel(i + 1) * dt;
    
    % Without heel support
    alpha_no_heel = -(g / effective_length_no_heel) * sin(theta_no_heel(i));
    omega_no_heel(i + 1) = omega_no_heel(i) + alpha_no_heel * dt;
    theta_no_heel(i + 1) = theta_no_heel(i) + omega_no_heel(i + 1) * dt;
end

% Plot results
figure;

% Plot angle over time
subplot(2, 1, 1);
plot(time, rad2deg(theta_heel), 'b', 'LineWidth', 1.5);
hold on;
plot(time, rad2deg(theta_no_heel), 'r', 'LineWidth', 1.5);
title('Inverted Pendulum around Ankle - Angle Over Time');
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('With Heel Support', 'Without Heel Support');
grid on;

% Plot angular velocity over time
subplot(2, 1, 2);
plot(time, omega_heel, 'b', 'LineWidth', 1.5);
hold on;
plot(time, omega_no_heel, 'r', 'LineWidth', 1.5);
title('Inverted Pendulum around Ankle - Angular Velocity Over Time');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('With Heel Support', 'Without Heel Support');
grid on;

disp('Simulation completed.');
