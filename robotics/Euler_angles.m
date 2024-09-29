% rotation matrices.
syms phi psi theta real;


% Rotation along x-axis
R1 = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];

% Rotation along y-axis;
R2 = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];

% Rotation along z-axis;
R3 = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];

Rotation_Matrix = R1 * R2 * R3;

z = simplify(Rotation_Matrix);

z

%
%[cos(phi)*cos(psi)*cos(theta) - sin(phi)*sin(psi), - cos(psi)*sin(phi) - cos(phi)*cos(theta)*sin(psi), cos(phi)*sin(theta)]
%[cos(phi)*sin(psi) + cos(psi)*cos(theta)*sin(phi),   cos(phi)*cos(psi) - cos(theta)*sin(phi)*sin(psi), sin(phi)*sin(theta)]
%[                            -cos(psi)*sin(theta),                                sin(psi)*sin(theta),          cos(theta)]
 