% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
% mu = 
% sig = 
% thre = 

mu = [  148.2378  139.3003   63.0501];

sig =   [   34.6886   25.2494  -43.5409;
   25.2494   25.9667  -41.3495;
  -43.5409  -41.3495   89.2830;];

P_inv = sig \ eye(size(sig));
mahal_thresh = 23; %7.815 3dof Chi-sq .1 or .05 conf 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
sample_ind = [];
for i = 1:length(I(:,1,1))
    for j = 1:length(I(1,:,1))
        x = squeeze(I(i,j,:));
        res = double(x) - mu';
        nis = res'*P_inv*res;
        if (nis < mahal_thresh)
            sample_ind = [sample_ind;[i,j]];
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.
bw = false(size(I(:,:,1)));
for i = 1:length(sample_ind(:,1))
    bw(sample_ind(i,1),sample_ind(i,2)) = true;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

% segI = 
% loc = 
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

bw_biggest = false(size(bw));

% http://www.mathworks.com/help/images/ref/bwconncomp.html
CC = bwconncomp(bw);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 

segI = bw_biggest;

% show the centroid
% http://www.mathworks.com/help/images/ref/regionprops.html
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
end
