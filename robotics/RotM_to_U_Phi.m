% mat

R = [-1/3 2/3 -2/3; 2/3 -1/3 -2/3; -2/3 -2/3 -1/3;];
RT = transpose(R)
% given a rotatioal matrix, find phi and u

phi = 2*pi-acos(trace(R)-1/2);
U = (1/2*sin(phi))*(R-RT);
disp(phi)

A = [ 0.3835 0.5710 -1.3954;0.5730 0.5919 0.0217;0.9287 -0.4119 1.1105;]
A = [0.2120  0.7743  0.5963;
     0.2120 -0.6321  0.7454;
     0.9540 -0.0316 -0.2981];

det(A), A*transpose(A), transpose(A)*A