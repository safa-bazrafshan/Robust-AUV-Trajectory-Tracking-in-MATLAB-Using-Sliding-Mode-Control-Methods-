function rmse = calculate_rmse(x_real, y_real, xd_func, yd_func, t_vec)
    % Calculates the RMSE between actual and desired trajectories

    N = length(t_vec);  % number of time steps
    sum_sq = 0;

    for i = 1:N
        xd = xd_func(t_vec(i));  % desired x at time t
        yd = yd_func(t_vec(i));  % desired y at time t

        % squared error between actual and desired positions
        sum_sq = sum_sq + (x_real(i) - xd)^2 + (y_real(i) - yd)^2;
    end

    % Root Mean Square Error
    rmse = sqrt(sum_sq / N);
end