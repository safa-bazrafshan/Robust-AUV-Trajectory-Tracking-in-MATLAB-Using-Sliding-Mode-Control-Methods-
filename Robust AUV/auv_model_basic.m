function dx = auv_model_basic(t, x, tau)

% Parameters
m = 50; I_z = 10;
X_u = -2; Y_v = -3; N_r = -1;

% States
u = x(1); v = x(2); r = x(3);
x_pos = x(4); y_pos = x(5); psi = x(6);

% Disturbances
d_u = 10 * sin(0.1 * t);
d_v = 8 * cos(0.05 * t);
d_r = 1 * sin(0.2 * t);

% Dynamics with disturbance
dx = zeros(6,1);
dx(1) = (tau(1) + d_u + X_u * u) / m;
dx(2) = (tau(2) + d_v + Y_v * v) / m;
dx(3) = (tau(3) + d_r + N_r * r) / I_z;

% Kinematics
dx(4) = u * cos(psi) - v * sin(psi);
dx(5) = u * sin(psi) + v * cos(psi);
dx(6) = r;

end