
function xhat = EKFstudent(t, z)
  % In this exercise, you will batch-process this data: you are provided a vector of timestamps (of length T), and a 3xT matrix of observations, z.
  xhat = zeros(2,length(t));
  P = 1e3*eye(2);

  Q = diag([30, 1]);
  R = diag([1 1 1]);
  D = 0.5; 

  for k=2:length(t)
      % time diff with previous sample. 
      dt = t(k)-t(k-1);
      
      A = [1 dt; 
          0 1];

      % cmpute xhat
      xhat(:,k) = A*xhat(:, k-1);

      P = A*P*A'+Q;

      h = [sind(xhat(1,k-1)); 
          cosd(xhat(1,k-1)); 
          xhat(2,k-1)];

      H = D*[cosd(xhat(1,k-1)) 0; 
          -sind(xhat(1,k-1)) 0 ; 
          0 1];

      K = P*H'*inv(H*P*H'+R);
      xhat(:,k) = xhat(:,k) + K*(z(:,k) - h);

      P = (eye(2)-K*H)*P;
  end
end
