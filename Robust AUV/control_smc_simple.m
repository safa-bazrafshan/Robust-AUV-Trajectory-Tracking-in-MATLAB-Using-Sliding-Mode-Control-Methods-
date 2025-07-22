clc; clear;

% Simulation time
t_span = 0:0.1:100;

% Initial state: [u; v; r; x; y; psi]
x0 = [0; 0; 0; 0; 0; 0];

% Desired trajectory functions
xd_func   = @(t) 10 * cos(0.05 * t);
yd_func   = @(t) 10 * sin(0.05 * t);
psid_func = @(t) 0.05 * t;

% Derivatives of desired trajectory
xd_dot    = @(t) -0.5 * sin(0.05 * t);
yd_dot    = @(t)  0.5 * cos(0.05 * t);
psid_dot  = @(t) 0.05;

% Controller gains
lambda = 1.5;
k_smc = [20; 10];  % Gain for sliding mode

% External disturbance
d_ext = [2; 0.5; 0.2];

% Preallocate state history
x_hist = zeros(length(t_span), 6);

% Initial state
x = x0;

% Integration loop
for i = 1:length(t_span)
    t = t_span(i);
    
    % Desired trajectory values
    xd = xd_func(t);
    yd = yd_func(t);
    psid = psid_func(t);
    
    xd_dot_val = xd_dot(t);
    yd_dot_val = yd_dot(t);
    psid_dot_val = psid_dot(t);
    
    % Current state
    u = x(1); v = x(2); r = x(3);
    x_pos = x(4); y_pos = x(5); psi = x(6);
    
    % Tracking error in position
    e_pos = [x_pos - xd; y_pos - yd];
    
    % Velocity in inertial frame
    v_inertial = [u * cos(psi) - v * sin(psi);
                  u * sin(psi) + v * cos(psi)];
    
    % Error derivative
    e_dot = v_inertial - [xd_dot_val; yd_dot_val];
    
    % Sliding surface
    s = e_dot + lambda * e_pos;
    
    % Control law (simplified SMC)
    Tx = -k_smc(1) * sign(s(1));
    Tz = -k_smc(2) * sign(s(2));
    tau = [Tx; Tz];
    
    % System dynamics integration
    dt = 0.1;
    dx = auv_model_dob(t, x, tau, d_ext);
    x = x + dx * dt;
    
    % Save current state
    x_hist(i,:) = x(:)';
end

% Generate desired path for plot
xd_path = arrayfun(xd_func, t_span);
yd_path = arrayfun(yd_func, t_span);

% Plot trajectory
figure;
plot(x_hist(:,4), x_hist(:,5), 'b', 'LineWidth', 2); hold on;
plot(xd_path, yd_path, 'r--', 'LineWidth', 2);
legend('AUV Trajectory with SMC', 'Desired Path');
xlabel('X position [m]');
ylabel('Y position [m]');
title('Step 3 – Trajectory Tracking with Simple SMC');
grid on;
axis equal;

% === Compute RMSE ===
x_real = x_hist(:,4);
y_real = x_hist(:,5);
t_vec = t_span;

rmse_smc = calculate_rmse(x_real, y_real, xd_func, yd_func, t_vec);
disp(['RMSE for SMC: ', num2str(rmse_smc)]);