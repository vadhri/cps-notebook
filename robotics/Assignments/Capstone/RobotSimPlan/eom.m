function qdd = eom(params, th, phi, dth, dphi, u)
  % This is the starter file for the week5 assignment

  % Provided params are
  % params.g: gravitational constant
  % params.mr: mass of the "rod"
  % params.ir: rotational inertia of the rod
  % params.d: distance of rod CoM from the wheel axis
  % params.r: wheel radius

  % Provided states are:
  % th: wheel angle (relative to body)
  % phi: body pitch
  % dth, dphi: time-derivatives of above
  % u: torque applied at the wheel

  qdd = [0;0];
  % THE STUDENT WILL FILL THIS OUT
  g = params.g;
  m = params.mr;
  i = params.ir;
  l = params.d;
  r = params.r;
  tau = u;
  
  A = sym(zeros(2,2));
  mr2 = m*r^2;
  mrlc = m*r*l*cos(phi);

  A(1,1) = mr2;
  A(1,2) = mr2 + mrlc;
  A(2,1) = mrlc + mr2;
  A(2,2) = 2*mrlc + i + m*l^2 + mr2;
  
  b = sym(zeros(2,1));
  b(1,1) = m*r*l*dphi^2 * sin(phi) + tau;
  b(2,1) = m*l*sin(phi)*(g + r*dphi^2);
  
  qdd = A\b;
end