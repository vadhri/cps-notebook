
function u = controller(params, t, X)
  u=[0; 0];

  % 1. write out the forward kinematics, such that p = FK(theta1, theta2)
  kp = 805;
  kd = 5;

  theta1 = X(1);
  theta2 = X(2);
  dt1 = X(3);
  dt2 = X(4);

  l = params.l;

  P = [l*cos(theta1)+l*cos(theta1 + theta2);l*sin(theta1)+l*sin(theta1+theta2)]

  % 2. Let e = p - params.traj(t) be the task-space error
  e = P - params.traj(t);

  % 3. Calculate the manipulator Jacobian J = d p / d theta
    J = [-l*sin(theta1) - l*sin(theta1+theta2) , -l*sin(theta1+theta2);
        l*cos(theta1) + l*cos(theta1+theta2) ,  l*cos(theta1+theta2)];

  % 4. Use a "natural motion" PD controller, u = - kp * J^T * e - kd * [dth1; dth2]
  u = -kp*J'*e - kd*[dt1; dt2];

end

