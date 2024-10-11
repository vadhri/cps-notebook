function F = EstimateFundamentalMatrix(X1, X2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2
N = size(X1,1);

x1 = X1(:,1);
y1 = X1(:,2);

x2 = X2(:,1);
y2 = X2(:,2);

matrix = [x2.*x1 x2.*y1 x2 y2.*x1 y2.*y1 y2 x1 y1 ones(N,1)];

[u d v] = svd(matrix);
F = reshape(v(:,9),3,3)';

[u d v] = svd(F,0);
F = u*diag([d(1,1) d(2,2) 0])*v';

end