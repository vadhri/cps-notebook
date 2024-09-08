%init

%parameters
a=1;
Tr=65;
Tdelta = 20;
parameters = [a;Tr;Tdelta];

%init
xInitial = 55;

%simulation horizon
T = 10;
J = 20;

%rule for jump 
rule = 1;

%solver
MaxStep = 1e-3;
RelTol = 1e-6;
