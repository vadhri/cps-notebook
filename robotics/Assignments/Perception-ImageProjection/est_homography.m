function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
A = [];

for i=1:4
    x1 = video_pts(i,1);
    x2 = video_pts(i,2);

    x1p = logo_pts(i,1);
    x2p = logo_pts(i,2);

    ax = [-x1 -x2 -1 0 0 0 x1*x1p x2*x1p x1p];
    ay = [0 0 0 -x1 -x2 -1 x1*x2p x2*x2p x2p];

    A = [A; ax];
    A = [A; ay];
end

[U, S, V] = svd(A);
H = reshape(V(:,end), 3,3)';

end

