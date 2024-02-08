function [distance, nearestPoint] = pointToLineSegmentDistance(point, lineStart, lineEnd)
    % Calculate vector representing the line segment
    lineVector = lineEnd - lineStart;
    
    % Calculate vector from line start to the point
    pointVector = point - lineStart;
    
    % Calculate projection of the point vector onto the line vector
    projection = dot(pointVector, lineVector) / norm(lineVector)^2 * lineVector;
    
    % Calculate the projected point
    projectedPoint = lineStart + projection;
    
    % Check if the projected point lies within the line segment
    if isInsideSegment(projectedPoint, lineStart, lineEnd)
        % Calculate distance between the projected point and the given point
        distance = norm(point - projectedPoint);
        nearestPoint = projectedPoint;
    else
        % If projected point is outside the line segment, use distance to
        % the closest endpoint
        distanceToStart = norm(point - lineStart);
        distanceToEnd = norm(point - lineEnd);
        if distanceToStart >= distanceToEnd
            distance = distanceToEnd;
            nearestPoint = lineEnd;
        else
            distance = distanceToStart;
            nearestPoint = lineStart;
        end
        
    end
end