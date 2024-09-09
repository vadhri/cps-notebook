%init
clear all;
clc;

%parameters
a=1;
Tr=65;
Tdelta = 20;
Tmax = 80;
Tmin = 70;

parameters = [a;Tr;Tdelta;Tmin;Tmax];

%init
xInitial = 55;
xInitialFsm = 0;

%simulation horizon
T = 10;
J = 20;

%rule for jump 
rule = 1;

%solver
MaxStep = 1e-3;
RelTol = 1e-6;
