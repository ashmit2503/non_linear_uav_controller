%Nonlinear Controller for UAV with Nested Saturation Technique
% Implements quadrotor control based on real-time embedded nonlinear control

clear; clc; close all;

%% Parameters of the UAV
m = 1;                 % Mass of the UAV [kg]
g = 9.81;              % Gravity [m/s^2]
I_x = 5e-3;            % Inertia along x-axis [kg*m^2]
I_y = 5e-3;            % Inertia along y-axis [kg*m^2]
Iz = 9e-3;             % Inertia along z-axis [kg*m^2]
I_r = 4e-5;            % Rotor inertia [kg*m^2]
l = 0.22;              % Arm length [m]
mu = 3e-6;             % Thrust coefficient
kappa = 1.5e-7;        % Drag coefficient

%% Controller Parameters
r1 = 3;  % Saturation limits for altitude control
r2 = 1.4;  % Saturation limits for yaw control
psi1 = 1.5; % Saturation for yaw rate
psi2 = 0.7; % Saturation for yaw angle

% Pitch and roll controller saturation and gains
a_theta = 1.7; b_theta = 0.82; c_theta = 0.36; d_theta = 0.16;
a_phi = 1.7; b_phi = 0.82; c_phi = 0.4; d_phi = 0.18;

% Gains for yaw control
k_r1 = 1; k_r2 = 2;
k_psi1 = 1; k_psi2 = 2;

% Nested saturation gains for theta and phi (pitch and roll)
k1_theta = 0.2; k2_theta = 0.5; k3_theta = 0.5; k4_theta = 1.8;
k1_phi = 0.5; k2_phi = 0.5; k3_phi = 0.5; k4_phi = 1;

%% Initial conditions (position and orientation)
x0 = 2; y0 = -3; z0 = 0; % Initial position [m]
phi0 = -0.7; theta0 = 0.8; psi0 = 0; % Initial orientation [rad]
xd = 0; yd = 0; zd = 1; % Desired position [m]
psi_d = 1;              % Desired yaw angle [rad]

%% Time setup
T = 100;                 % Simulation time [s]
dt = 0.01;              % Time step [s]
time = 0:dt:T;
n = length(time);

%% State Vector Initialization
state = [x0; y0; z0; phi0; theta0; psi0; 0; 0; 0; 0; 0; 0]; % [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot]

% Pre-allocate arrays for storing results
pos_data = zeros(n, 3);    % [x, y, z]
orient_data = zeros(n, 3); % [phi, theta, psi]
control_inputs = zeros(n, 4); % [u, tau_phi, tau_theta, tau_psi]

%% Simulation Loop
for i = 1:n
    % Extract the state variables
    x = state(1); y = state(2); z = state(3);
    phi = state(4); theta = state(5); psi = state(6);
    x_dot = state(7); y_dot = state(8); z_dot = state(9);
    phi_dot = state(10); theta_dot = state(11); psi_dot = state(12);
    
    %% Altitude control (z)
    r = -nested_saturation(k_r1*z_dot + nested_saturation(k_r2*z_dot + k_r1*k_r2*(z-zd), r2), r1);
    u = (m*r + m*g) / (cos(theta) * cos(phi)); % Thrust for altitude
    
   %% Yaw control (psi)
tau_psi = -nested_saturation(k_psi1*psi_dot + nested_saturation(k_psi2*psi_dot + k_psi1*k_psi2*(psi - psi_d), psi2), psi1);
    
 %% Pitch and Roll Control
% Roll (phi) - Using the new nested saturation formula
tau_phi = -nested_saturation(k1_phi*phi_dot + ...
    nested_saturation(k2_phi*(phi_dot + k1_phi*phi) + ...
    nested_saturation(k3_phi*(phi_dot + (k1_phi + k2_phi)*phi + k2_phi*k1_phi*y_dot/g) + ...
    nested_saturation(k4_phi*(phi_dot + (k1_phi + k2_phi + k3_phi)*phi + ...
    (k2_phi*k1_phi + k3_phi*k1_phi + k3_phi*k2_phi)*y_dot/g + ...
    k1_phi*k2_phi*k3_phi*y/g), d_phi), c_phi), b_phi), a_phi);

% Pitch (theta) - Using the new nested saturation formula
tau_theta = -nested_saturation(k1_theta*theta_dot + ...
    nested_saturation(k2_theta*(theta_dot + k1_theta*theta) + ...
    nested_saturation(k3_theta*(theta_dot + (k1_theta + k2_theta)*theta - k2_theta*k1_theta*x_dot/g) + ...
    nested_saturation(k4_theta*(theta_dot + (k1_theta + k2_theta + k3_theta)*theta - ...
    (k2_theta*k1_theta + k3_theta*k1_theta + k3_theta*k2_theta)*x_dot/g - ...
    k3_theta*k2_theta*k1_theta*x/g), d_theta), c_theta), b_theta), a_theta);
    % Control torques
tau = [tau_phi; tau_theta; tau_psi]; 
    
  %% Dynamics Update
    % Position dynamics from eq. (16)
    x_ddot = -u * sin(theta)/m;
    y_ddot = u * sin(phi) * cos(theta)/m;
    z_ddot = u * cos(phi) * cos(theta)/m - g;
    
    % Angular dynamics from eq. (16)
    phi_ddot = tau_phi;
    theta_ddot = tau_theta;
    psi_ddot = tau_psi;
    
    % Update positions
    state(1) = state(1) + state(7) * dt + 0.5 * x_ddot * dt^2;  % x
    state(2) = state(2) + state(8) * dt + 0.5 * y_ddot * dt^2;  % y
    state(3) = state(3) + state(9) * dt + 0.5 * z_ddot * dt^2;  % z
    
    % Update velocities
    state(7) = state(7) + x_ddot * dt;  % x_dot
    state(8) = state(8) + y_ddot * dt;  % y_dot
    state(9) = state(9) + z_ddot * dt;  % z_dot
    
    % Update angles
    state(4) = state(4) + state(10) * dt + 0.5 * phi_ddot * dt^2;    % phi
    state(5) = state(5) + state(11) * dt + 0.5 * theta_ddot * dt^2;  % theta
    state(6) = state(6) + state(12) * dt + 0.5 * psi_ddot * dt^2;    % psi
    
    % Update angular rates
    state(10) = state(10) + phi_ddot * dt;    % phi_dot
    state(11) = state(11) + theta_ddot * dt;  % theta_dot
    state(12) = state(12) + psi_ddot * dt;    % psi_dot
    
    % Normalize angles to [-pi, pi]
    state(4:6) = mod(state(4:6) + pi, 2*pi) - pi;
    
    %% Store data for plotting
    pos_data(i, :) = state(1:3);    % Store position [x, y, z]
    orient_data(i, :) = state(4:6); % Store orientation [phi, theta, psi]
    control_inputs(i, :) = [u, tau_phi, tau_theta, tau_psi]; % Store control inputs
end

%% Plotting Results
time_limit = 20;
idx = find(time <= time_limit, 1, 'last');

% 1. Position (x, y, z)
figure;
subplot(2,2,1);
plot(time, pos_data(:,1), 'r', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('x [m]');
title('Position in x-direction');
grid on;

subplot(2,2,2);
plot(time, pos_data(:,2), 'g', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('y [m]');
title('Position in y-direction');
grid on;

subplot(2,2,[3,4]);
plot(time(1:idx), pos_data(1:idx,3), 'b', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('z [m]');
title('Position in z-direction');
grid on;

% 2. Orientation (phi, theta, psi)
figure;
subplot(2,2,1);
plot(time, orient_data(:,1), 'r', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\phi [rad]');
title('Roll Angle \phi');
grid on;

subplot(2,2,2);
plot(time, orient_data(:,2), 'g', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\theta [rad]');
title('Pitch Angle \theta');
grid on;

subplot(2,2,[3,4]);
plot(time(1:idx), orient_data(1:idx,3), 'b', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\psi [rad]');
title('Yaw Angle \psi');
grid on;

% 3. Control Inputs (u, tau_phi, tau_theta, tau_psi)
figure;
subplot(2,2,1);
plot(time(1:idx), control_inputs(1:idx,1), 'b', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('u [N]');
title('Vertical Thrust u');
grid on;

subplot(2,2,2);
plot(time, control_inputs(:,2), 'r', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\tau_\phi [Nm]');
title('Roll Torque \tau_\phi');
grid on;

subplot(2,2,3);
plot(time, control_inputs(:,3), 'g', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\tau_\theta [Nm]');
title('Pitch Torque \tau_\theta');
grid on;

subplot(2,2,4);
plot(time(1:idx), control_inputs(1:idx,4), 'm', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\tau_\psi [Nm]');
title('Yaw Torque \tau_\psi');
grid on;

%% Nested Saturation Function (Helper Function)
function sat_val = nested_saturation(value, sat_level)
    % Nested saturation function to limit values within bounded levels
    sat_val = sign(value) * min(abs(value), sat_level);
end
