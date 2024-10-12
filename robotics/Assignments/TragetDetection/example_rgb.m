% Robotics: Estimation and Learning 
% WEEK 3
% 
% This is an example code for collecting ball sample colors using roipoly
close all

imagepath = './train';
Samples = [];
for k=1:2
    % Load image
    I = imread(sprintf('%s/%03d.png',imagepath,k));
    
    % You may consider other color space than RGB
    R = I(:,:,1);
    G = I(:,:,2);
    B = I(:,:,3);
    
    % Collect samples 
    disp('');
    disp('INTRUCTION: Click along the boundary of the ball. Double-click when you get back to the initial point.')
    disp('INTRUCTION: You can maximize the window size of the figure for precise clicks.')
    figure(1), 
    mask = roipoly(I); 
    figure(2), imshow(mask); title('Mask');
    sample_ind = find(mask > 0);
    
    R = R(sample_ind);
    G = G(sample_ind);
    B = B(sample_ind);
    
    Samples = [Samples; [R G B]];
    
    disp('INTRUCTION: Press any key to continue. (Ctrl+c to exit)')
    pause
end

% visualize the sample distribution
figure, 
scatter3(Samples(:,1),Samples(:,2),Samples(:,3),'.');
title('Pixel Color Distribubtion');
xlabel('Red');
ylabel('Green');
zlabel('Blue');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [IMPORTANT]
%
% Now choose you model type and estimate the parameters (mu and Sigma) from
% the sample data.
%

output = fitgmdist([double(Samples(:,1)) double(Samples(:,2)) double(Samples(:,3))], 1);

disp(output.mu);
disp(output.Sigma);

  % 148.2378  139.3003   63.0501
  % 
  %  34.6886   25.2494  -43.5409
  %  25.2494   25.9667  -41.3495
  % -43.5409  -41.3495   89.2830