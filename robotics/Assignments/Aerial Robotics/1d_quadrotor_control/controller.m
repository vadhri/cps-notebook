function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;
m=params.mass
g=params.gravity

% FILL IN YOUR CODE HERE
kp=100;
kd=15;

u = m*(sum([kp;kd].*(s_des-s))+g);

if u>params.u_max
    u=params.u_max;
end

end

