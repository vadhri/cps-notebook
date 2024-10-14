
function u = controller(params, t, X)
  % You have full state feedback available
  K = [-1.0000 -113.1952 -1.2465  -13.9340];
  u = -K*X;
end

