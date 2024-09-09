%init 
clear all;
clc;

%simuation horizon.
T=10;
x0=1;
J=20;

%rule
rule=1;

%solver
MaxStep=1e-3;
RelTol=1e-6;