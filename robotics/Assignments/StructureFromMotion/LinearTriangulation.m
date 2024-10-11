function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 3) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 3) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points


N = size(x1,1); % num of correspondences
X = zeros(N,3);

P1 = K*R1*[eye(3), -C1];
P2 = K*R2*[eye(3), -C2];

for i = 1:N
    X1=[x1(i,:) ,1]; 
    X2=[x2(i,:) ,1];  
    S1 =[0 -X1(3) X1(2); X1(3) 0 -X1(1); -X1(2) X1(1) 0];
    S2 =[0 -X2(3) X2(2); X2(3) 0 -X2(1); -X2(2) X2(1) 0];
    F=[S1*P1;S2*P2];
    [U,D,V] = svd(F);
    X_prime = (V(:,end)/V(end,end))';
    X(i,:) = X_prime(1:3);
end
end

