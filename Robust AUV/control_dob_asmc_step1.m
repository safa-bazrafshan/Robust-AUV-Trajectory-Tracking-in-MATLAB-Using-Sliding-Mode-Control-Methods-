clc; clear;

% Time span
t_span = [0 100];

% Initial state: [u; v; r; x; y; psi]
x0 = [0; 0; 0; 0; 0; 0];

% External disturbance
d_ext = [2; 0.5; 0.2];

% Desired trajectory (circular path)
xd_func = @(t) 10 * cos(0.05 * t);
yd_func = @(t) 10 * sin(0.05 * t);
psid_func = @(t) 0.05 * t;

% Prealloc arrays
n = length(t_span(1):1:t_span(2));
x_state = zeros(n, 6);
xd_desired = zeros(n, 1);
yd_desired = zeros(n, 1);

% (For now, tau = zeros — no controller yet)
tau_dummy = [0; 0];

% Define system for ode45
auv_dyn = @(t, x) auv_model_dob(t, x, tau_dummy, d_ext);

% Run simulation
[t, x] = ode45(auv_dyn, t_span, x0);

% Desired path for plotting
xd = xd_func(t);
yd = yd_func(t);

% Plot
figure;
plot(x(:,4), x(:,5), 'b', 'LineWidth', 2); hold on;
plot(xd, yd, 'r--', 'LineWidth', 2);
legend('AUV Actual Path (Open-loop)', 'Desired Path');
xlabel('X position [m]');
ylabel('Y position [m]');
title('Step 1 – Trajectory Tracking Without Controller');
grid on;
axis equal;