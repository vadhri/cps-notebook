% init 
T = 10;
J = 20;

% solver
MaxStep = 1e-3;
RelTol = 1e-6;

%rule 
rule=1;

% min-max 
Tmin = 70;
Tmax = 80;

%initial temp
x0 = 0;

parameters=[Tmin;Tmax];