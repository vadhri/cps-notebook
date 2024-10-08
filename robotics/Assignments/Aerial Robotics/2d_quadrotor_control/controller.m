function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

m = params.mass;
g = params.gravity;
ixx = params.Ixx;
L = params.arm_length;

% Proportional and Derivative Gains
Kp_z = 90;
Kd_z = 10;

Kp_y = 25;
Kd_y = 5;

Kp_a = 1500;
Kd_a = 10;

y = state.pos(1);
z = state.pos(2);

ydot = state.vel(1);
zdot = state.vel(2);

phi = state.rot(1);
phidot = state.omega(1);

ydes = des_state.pos(1);
zdes = des_state.pos(2);

ydotdes = des_state.vel(1);
zdotdes = des_state.vel(2);

y_ddot = des_state.acc(1);
z_ddot = des_state.acc(2);

phi_c = -(1/g)*(y_ddot + Kd_y*(ydotdes-ydot + Kp_y*(ydes-y)));
u1 = m*(g + z_ddot + Kd_z*(zdotdes-zdot) + Kp_z*(zdes-z));
u2 = ixx*(Kd_a*(-phidot) + Kp_a*(phi_c-phi));

% FILL IN YOUR CODE HERE

end
