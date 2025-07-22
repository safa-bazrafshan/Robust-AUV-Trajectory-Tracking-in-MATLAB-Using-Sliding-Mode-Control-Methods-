clc; clear;

% Time setup
t_span = 0:0.1:100;
x0 = [0; 0; 0; 0; 0; 0];
x = x0;

% Gains
lambda1 = 1.5;
lambda2 = 2.0;
K = [1.0; 1.0];
gamma = [0.3; 0.3];

% Disturbance
d_ext = [2; 0.5; 0.2];

% Save history
x_hist = zeros(length(t_span), 6);
K_hist = zeros(length(t_span), 2);

% Desired trajectory
xd = @(t) 10 * cos(0.05 * t);
yd = @(t) 10 * sin(0.05 * t);
xd_dot = @(t) -0.5 * sin(0.05 * t);
yd_dot = @(t)  0.5 * cos(0.05 * t);
psid = @(t) 0.05 * t;

% Loop
for i = 1:length(t_span)
    t = t_span(i);
    
    % Desired
    xd_val = xd(t);  yd_val = yd(t);
    xd_dot_val = xd_dot(t);  yd_dot_val = yd_dot(t);
    psi_d = psid(t);
    
    % Current state
    u = x(1); v = x(2); r = x(3);
    x_pos = x(4); y_pos = x(5); psi = x(6);
    
    % Error
    e_pos = [x_pos - xd_val; y_pos - yd_val];
    
    % Convert body to inertial
    J = [cos(psi) -sin(psi); sin(psi) cos(psi)];
    v_inertial = J * [u; v];
    e_dot = v_inertial - [xd_dot_val; yd_dot_val];

    % Backstepping virtual control
    alpha = [xd_dot_val; yd_dot_val] - lambda1 * e_pos;
    z2 = v_inertial - alpha;
    
    % Adaptive gain update
    K_dot = gamma .* abs(z2);
    K = min(K + K_dot * 0.1, [15; 15]);

    % Control law
    tau = -K .* tanh(3 * z2) + d_ext(1:2);  % tanh = smooth SMC + adaptive

    % System dynamics
    dt = 0.1;
    dx = auv_model_dob(t, x, tau, d_ext);
    x = x + dx * dt;

    % Save
    x_hist(i,:) = x(:)';
    K_hist(i,:) = K(:)';
end

% Desired path
xd_path = xd(t_span);
yd_path = yd(t_span);

% Plot
figure;
plot(x_hist(:,4), x_hist(:,5), 'b', 'LineWidth', 2); hold on;
plot(xd_path, yd_path, 'r--', 'LineWidth', 2);
legend('AUV Trajectory (ABSMC)', 'Desired Path');
xlabel('X [m]'); ylabel('Y [m]');
title('Adaptive Backstepping Sliding Mode Control');
grid on; axis equal;

t_vec = t_span;  % Time vector

% Actual trajectory (from simulation result)
x_real = x_hist(:,4);  % x position
y_real = x_hist(:,5);  % y position

% Desired trajectory (functions of time)
xd_func = @(t) 10 * cos(0.05 * t);
yd_func = @(t) 10 * sin(0.05 * t);

% Call RMSE calculation function
rmse_abs = calculate_rmse(x_real, y_real, xd_func, yd_func, t_vec);

% Show result
disp(['RMSE for ABSMC: ', num2str(rmse_abs)]);