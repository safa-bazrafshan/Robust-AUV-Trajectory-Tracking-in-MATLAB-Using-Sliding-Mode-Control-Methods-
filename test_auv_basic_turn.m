% File: test_auv_model.m
% Description: Simulate the AUV basic model

% Initial conditions
x0 = [0; 0; 0; 0; 0; 0];  % [u; v; r; x; y; psi]
tspan = [0 30];           % simulate for 30 seconds

% Constant input (forward thrust only)
tau = [50; 0; 5];         % [thrust + yaw torque]

% Use anonymous function to pass tau
[t, x] = ode45(@(t,x) auv_model_basic(t, x, tau), tspan, x0);

% Plot X and Y positions separately
figure;
subplot(2,1,1);
plot(t, x(:,4), 'r', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('X Position (m)');
title('X Position vs Time'); grid on;

subplot(2,1,2);
plot(t, x(:,5), 'b', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Y Position (m)');
title('Y Position vs Time'); grid on;