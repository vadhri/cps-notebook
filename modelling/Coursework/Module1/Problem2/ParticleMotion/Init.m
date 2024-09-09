%init
clear all;
clc;

%z1 (horizontal position), z2 (horizontal velocity)
z1 = 1
z2 = 0

%initial position
xInitial=[z1;z2];

T=10;

%simulation params
MaxStep = 1e-3;
RelTol = 1e-6;