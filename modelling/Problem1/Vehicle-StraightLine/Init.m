%init
clear all;
clc;

%directional vector
angle = -pi/4
r = [cos(angle);sin(angle)];
parameters = [r(1); r(2)];

%initial position
xInitial=[1;1];

T=10;

%simulation params
MaxStep = 1e-3;
RelTol = 1e-6;