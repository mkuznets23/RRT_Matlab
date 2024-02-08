function inside = isInsideSegment(point, segmentStart, segmentEnd)
    % Check if the point is inside the line segment defined by segmentStart and segmentEnd
    inside = dot(point - segmentStart, segmentEnd - segmentStart) >= 0 && ...
             dot(point - segmentEnd, segmentStart - segmentEnd) >= 0;
end