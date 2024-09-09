% Parameters
T_r = 0;  % Room temperature
T_target = 75;  % Target temperature (midpoint of [70, 80])
K_p = 1.0;  % Proportional gain
K_i = 0.1;  % Integral gain
K_d = 0.01;  % Derivative gain
dt = 0.1;  % Time step
time = 0:dt:50;  % Simulation time
n = length(time);  % Number of time steps

% Initial conditions
initial_conditions = [0, 120];  % Two initial temperatures
results = cell(2,1);  % Store results for both initial conditions

for j = 1:length(initial_conditions)
    T = zeros(1, n);  % Temperature array
    u = zeros(1, n);  % Control input array
    e_int = 0;  % Integral term
    e_prev = 0;  % Previous error
    T(1) = initial_conditions(j);  % Set initial temperature
    
    for i = 2:n
        e = T_target - T(i-1);  % Calculate error
        e_int = e_int + e * dt;  % Update integral term
        e_der = (e - e_prev) / dt;  % Calculate derivative term
        
        % PID control
        u(i) = K_p * e + K_i * e_int + K_d * e_der;
        
        % Update temperature (simple model)
        % Assume that control input affects temperature directly
        T(i) = T(i-1) + u(i) * dt; 
        
        % Update previous error
        e_prev = e;
    end
    
    results{j}.time = time;
    results{j}.temperature = T;
end

% Plot results
figure;
hold on;
plot(results{1}.time, results{1}.temperature, 'b', 'DisplayName', 'Initial Temp = 0');
plot(results{2}.time, results{2}.temperature, 'r', 'DisplayName', 'Initial Temp = 120');
xlabel('Time');
ylabel('Temperature');
title('Temperature as a Function of Time with PID Control');
legend;
grid on;
hold off;
