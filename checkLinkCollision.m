function result = checkLinkCollision(linkStart,linkEnd,obstaclePos, obstacleRad)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    [distance, ~] = pointToLineSegmentDistance(obstaclePos, linkStart, linkEnd);
    if distance <= obstacleRad*1.1
        result = 1;
    else
        result = 0;
    end
end

