clc; clear;

% Initial state: [u0; v0; r0; x0; y0; psi0]
x0 = [0; 0; 0; 0; 0; 0];

% Simulation time
t_span = [0 100];

% Constant input (open-loop thrust + torque)
tau = [30; 2];  % [Tx; Tz]

% Constant external disturbance
d_ext = [2; 0.5; 0.2];  % small unknown water current

% Define the system as an anonymous function for ode45
auv_sys = @(t, x) auv_model_dob(t, x, tau, d_ext);

% Run simulation
[t, x] = ode45(auv_sys, t_span, x0);

% Plot trajectory
figure;
plot(x(:,4), x(:,5), 'b', 'LineWidth', 2);
xlabel('X position [m]');
ylabel('Y position [m]');
title('AUV Trajectory (Open-loop)');
grid on;
axis equal;