function u = controllerNoisyEnc(params, t, obs, th, dth)
  % This is the starter file for the week5 assignment
  % Now you only receive noisy measurements for phi, and must use your EKF from week 3 to filter the data and get an estimate of the state
  % obs = [ay; az; gx] with a* in units of g's, and gx in units of rad/s

  % This template code calls the function EKFupdate that you must complete below
  xhat = EKFupdate(params, t, obs);
  phi = xhat(1);
  phidot = xhat(2);
  x=params.r*(th+phi);
  xdot=params.r*(dth+phidot);
  
  kp1=0.15;
  kd1=0.15;
  kp2=0.1;
  kd2=0.02;

  utmp=kp1*(params.traj(t)-x)+kd1*(0-xdot);
  u=-kp2*sin(phi-asin(utmp))-kd2*phidot;
end

function xhatOut = EKFupdate(params, t, z)
  persistent P xhat time_prev k
  if isempty(P)
    P=1e3*eye(2);
    xhat = zeros(2,length(t));
    time_prev = t;
    k = 1;
  end

  k = k+1;
  Q = diag([10000, 1]);
  R = diag([0.005 0.05 5]);
  D = 0.5; 

  % time diff with previous sample. 
  dt = t-time_prev;
  
  A = [1 dt; 
      0 1];

  % cmpute xhat
  xhat(:,k) = A*xhat(:, k-1);

  P = A*P*A'+Q;

  h = [sin(xhat(1,k-1)); 
      cos(xhat(1,k-1)); 
      xhat(2,k-1)];

  H = [cos(xhat(1,k-1)) 0; 
      -sin(xhat(1,k-1)) 0 ; 
      0 1];

  K = P*H'*inv(H*P*H'+R);
  xhat(:,k) = xhat(:,k) + K*(z - h);

  P = (eye(2)-K*H)*P;
  xhatOut = xhat(:, k);  
end

