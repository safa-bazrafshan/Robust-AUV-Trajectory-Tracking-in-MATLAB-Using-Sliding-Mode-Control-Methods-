clc; clear;

% Smoothed sign function using tanh
smooth_sign = @(x) tanh(3 * x);

% Simulation parameters
t_span = 0:0.1:100;
x0 = [0; 0; 0; 0; 0; 0];   % Initial state: [u; v; r; x; y; psi]
K = [1; 1];                % Initial adaptive gains
gamma = [0.5; 0.5];        % Adaptation rates
lambda = 1.5;              % Sliding surface gain
dt = 0.1;                  % Time step

% Desired trajectory functions
xd_func   = @(t) 10 * cos(0.05 * t);
yd_func   = @(t) 10 * sin(0.05 * t);
psid_func = @(t) 0.05 * t;

% Derivatives of desired trajectory
xd_dot    = @(t) -0.5 * sin(0.05 * t);
yd_dot    = @(t)  0.5 * cos(0.05 * t);
psid_dot  = @(t) 0.05;

% External disturbance (constant)
d_ext = [2; 0.5; 0.2];

% Data storage
x_hist = zeros(length(t_span), 6);   % state history
K_hist = zeros(length(t_span), 2);   % adaptive gain history
x = x0;

% Simulation loop
for i = 1:length(t_span)
    t = t_span(i);

    % Desired position and velocity
    xd = xd_func(t);
    yd = yd_func(t);
    xd_dot_val = xd_dot(t);
    yd_dot_val = yd_dot(t);

    % Current state
    u = x(1); v = x(2); r = x(3);
    x_pos = x(4); y_pos = x(5); psi = x(6);

    % Position and velocity errors
    e_pos = [x_pos - xd; y_pos - yd];
    v_inertial = [u * cos(psi) - v * sin(psi);
                  u * sin(psi) + v * cos(psi)];
    e_dot = v_inertial - [xd_dot_val; yd_dot_val];

    % Sliding surface
    s = e_dot + lambda * e_pos;

    % Adaptive gain update
    K_dot = gamma .* abs(s);
    K = min(K + K_dot * dt, [15; 15]);  % Limit max gain

    % Control input with smooth sign
    d_hat = d_ext(1:2);  % estimated disturbance (constant here)
    tau = -K .* arrayfun(smooth_sign, s) + d_hat;

    % System dynamics
    dx = auv_model_dob(t, x, tau, d_ext);
    x = x + dx * dt;

    % Save data
    x_hist(i,:) = x(:)';
    K_hist(i,:) = K(:)';
end

% Extract final path
x_real = x_hist(:,4);
y_real = x_hist(:,5);

% Calculate RMSE
rmse_adaptive = calculate_rmse(x_real, y_real, xd_func, yd_func, t_span);
fprintf('RMSE for Adaptive SMC: %.4f\n', rmse_adaptive);

% Desired path for plot
xd_path = xd_func(t_span);
yd_path = yd_func(t_span);

% Plot trajectory
figure;
plot(x_real, y_real, 'b', 'LineWidth', 2); hold on;
plot(xd_path, yd_path, 'r--', 'LineWidth', 2);
legend('AUV Trajectory (Adaptive SMC)', 'Desired Path');
xlabel('X [m]'); ylabel('Y [m]');
title('Adaptive Sliding Mode Control with Disturbance Observer');
grid on; axis equal;