phi=2*pi/3;
U = [1/sqrt(3); 1/sqrt(3); 1/sqrt(3)];
Uhat = [0 -U(3) U(2); U(3) 0 -U(1); -U(2) U(1) 0];

Rotation = eye(3)*cos(phi)+U*transpose(U)*(1-cos(phi))+Uhat*sin(phi);
round(Rotation)
