function flag = triangle_intersection(P1, P2)
% triangle_intersection : returns true if the triangles overlap or intersect
%                         and false otherwise.

% Helper function to check for overlap on a given axis
    function is_disjoint = check_separation_axis(P1, P2, edge)
        % Compute the normal to the edge (perpendicular vector)
        normal = [-edge(2), edge(1)]; % Rotate the edge 90 degrees to get the normal
        
        % Project both triangles onto the normal axis
        proj_P1 = P1 * normal';
        proj_P2 = P2 * normal';
        
        % Find the min and max projection for both triangles
        min_P1 = min(proj_P1);
        max_P1 = max(proj_P1);
        min_P2 = min(proj_P2);
        max_P2 = max(proj_P2);
        
        % Check if the projections are disjoint
        is_disjoint = max_P1 < min_P2 || max_P2 < min_P1;
    end

% *******************************************************************
% Check for intersection using the Separating Axis Theorem (SAT)

% Get the edges of both triangles
edges_P1 = [P1(2,:) - P1(1,:); P1(3,:) - P1(2,:); P1(1,:) - P1(3,:)];
edges_P2 = [P2(2,:) - P2(1,:); P2(3,:) - P2(2,:); P2(1,:) - P2(3,:)];

% Check all edges of P1 for separating axis
for i = 1:3
    if check_separation_axis(P1, P2, edges_P1(i,:))
        flag = false;
        return;
    end
end

% Check all edges of P2 for separating axis
for i = 1:3
    if check_separation_axis(P1, P2, edges_P2(i,:))
        flag = false;
        return;
    end
end

% If no separating axis is found, the triangles intersect
flag = true;
% *******************************************************************
end
