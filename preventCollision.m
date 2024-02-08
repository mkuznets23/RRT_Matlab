function newNode_val = preventCollision(obstacles,nearestPoint, newNode_val)
%here we're checking if the node collides with obstacle
%and move it if it does it gets moved toward the nearest point on tree
%until there is no longer any collision
    for i = 1:length(obstacles)
        obstacleCenter = obstacles{i}.center;
        obstacleRadius = obstacles{i}.radius;
        distanceToObstacleCenter = pointToLineSegmentDistance(obstacleCenter, nearestPoint, newNode_val);
        while distanceToObstacleCenter < obstacleRadius
            %collision occurs, so must move node
            newNode_val = newNode_val -(newNode_val-nearestPoint)/norm(newNode_val-nearestPoint);
            distanceToObstacleCenter = pointToLineSegmentDistance(obstacleCenter, nearestPoint, newNode_val);
        end
    end
end

