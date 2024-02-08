function [smallestDistanceIndex,nearestPoint] = findNearestEdge(children,parents,newNode_val)
% find nearest edge to new node by iterating through parents
% and children arrays
    smallestDistanceIndex = 1;
    smallestDistance = 10000;
    nearestPoint = [0,0];
    for i = 1:length(children)
        [dist, nearPoint] = pointToLineSegmentDistance...
            (newNode_val, children(i).Value, parents(i).Value);
        if dist < smallestDistance
            smallestDistanceIndex = i; 
            smallestDistance = dist;
            nearestPoint = nearPoint;
        end    
    end
end

