% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)

myResol = param.resol;
% % the initial map size in pixels
myMap = zeros(param.size);
map = myMap;

% % the origin of the map in pixels
myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
for j = 1:N % for each time,
    for i=1:size(ranges(:,j))
        d = ranges(i,j);
        ta = scanAngles(i)+pose(3,j);
        occ = d*[cos(ta); -sin(ta)]+pose(1:2,j);
        movement = myorigin + ceil(myResol * occ);
        indices = ceil(myResol * pose(1:2,j)) + myorigin;

        % Find occupied-measurement cells and free-measurement cells
        [freex, freey] = bresenham(indices(1), indices(2), movement(1), movement(2));
        free = sub2ind(size(myMap),freey,freex);
        % set end point value
        map(movement(2),movement(1)) = 1;
        % set free cell values
        map(free) = 0;
    end 

    % Update the log-odds
    occ = find(map > 0);
    free = setdiff(1:param.size(1)*param.size(2),occ);
    myMap(free) = myMap(free) - lo_free;
    myMap(occ)  = myMap(occ) + lo_occ;

    % Saturate the log-odd values
    over_min = find(myMap < lo_min);
    over_max = find(myMap > lo_max);
    myMap(over_min) = lo_min;
    myMap(over_max) = lo_max;

    % Visualize the map as needed
    %image(myMap)
end

end

