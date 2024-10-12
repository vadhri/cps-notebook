function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)
    dt = 0.033
    
    sigma_m = diag([1e1 1e1 1e1 1e1]);
    sigma_o = diag([0.01 0.01]);

    A = [1 dt 0  0;
         0  1 0  0; 
         0  0 1 dt;
         0  0 0  1];

    C = [1 0 0 0;
         0 0 1 0];

    % Check if the first time running this function
    if previous_t<0
        state = [x, 0, y, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates

    P=A*param.P*A'+sigma_m;
    R=C*P*C'+sigma_o;
    K=P*C'*inv(R+C*P*C');
    param.P=P-K*C*P;

    output=A*state'+K*([x;y]-C*A*state');

    predictx=output(1)+output(2)*10*dt;
    predicty=output(3)+output(4)*10*dt;

    state = [x, output(2), y, output(4)];
end
