function dx = auv_model_dob(t, x, tau, d_ext)
    % State vector: x = [u; v; r; x_pos; y_pos; psi]
    % Control input: tau = [Tx; Tz] (surge thrust and yaw torque)
    % External disturbance: d_ext = [du; dv; dr]

    % Extract states
    u = x(1); v = x(2); r = x(3);
    x_pos = x(4); y_pos = x(5); psi = x(6);

    % AUV parameters
    m = 50;       % Mass (kg)
    Iz = 10;      % Yaw moment of inertia (kg*m^2)
    Xu = 8;       % Surge damping
    Yv = 15;      % Sway damping
    Nr = 5;       % Yaw damping

    % Control input
    Tx = tau(1);  % Surge thrust
    Tz = tau(2);  % Yaw torque

    % External disturbance
    du = d_ext(1);
    dv = d_ext(2);
    dr = d_ext(3);

    % Dynamic equations
    du_dt = (1/m) * (Tx - Xu * u + du);
    dv_dt = (1/m) * (-Yv * v + dv);
    dr_dt = (1/Iz) * (Tz - Nr * r + dr);

    % Kinematic equations
    dx_pos = u * cos(psi) - v * sin(psi);
    dy_pos = u * sin(psi) + v * cos(psi);
    dpsi = r;

    % State derivatives
    dx = [du_dt; dv_dt; dr_dt; dx_pos; dy_pos; dpsi];
end