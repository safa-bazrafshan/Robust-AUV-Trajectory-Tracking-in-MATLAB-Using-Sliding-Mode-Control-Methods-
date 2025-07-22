    
    % --- Parameters ---
    m = 50;       % mass (kg)
    Iz = 10;      % moment of inertia (kg.m^2)
    Xu = -2;      % surge damping
    Yv = -3;      % sway damping
    Nr = -1;      % yaw damping

    % Mass and inertia matrix
    M = diag([m, m, Iz]);

    % Velocity state
    u = x(1); v = x(2); r = x(3);
    x_pos = x(4); y_pos = x(5); psi = x(6);

    % Velocity vector
    nu = [u; v; r];

    % Coriolis matrix
    C = [0, 0, -m*v;
         0, 0,  m*u;
         m*v, -m*u, 0];

    % Damping matrix
    D = -diag([Xu, Yv, Nr]);

    % Rotation matrix (from body to inertial)
    J = [cos(psi), -sin(psi), 0;
         sin(psi),  cos(psi), 0;
         0,         0,        1];

    % Dynamics
    nudot = M \ (tau - C*nu - D*nu);
    etadot = J * nu;

    dx = [nudot; etadot];

% Disturbances (e.g., from ocean currents)
d_u = 10 * sin(0.1 * t);     % Surge disturbance
d_v = 8 * cos(0.05 * t);  % Sway disturbance
d_r = 1 * sin(0.2 * t);   % Yaw rate disturbance

% Equations of motion with disturbance
dx(1) = (tau(1) + d_u + X_u * u) / m;     % surge
dx(2) = (tau(2) + d_v + Y_v * v) / m;     % sway
dx(3) = (tau(3) + d_r + N_r * r) / I_z;   % yaw

% Kinematics
dx(4) = u * cos(x(6)) - v * sin(x(6));  % x_dot
dx(5) = u * sin(x(6)) + v * cos(x(6));  % y_dot
dx(6) = r;                              % yaw angle
end
