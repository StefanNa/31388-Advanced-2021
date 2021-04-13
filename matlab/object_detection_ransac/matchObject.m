function objectNumber = matchObject(sortedLengths,objectLengths, lineAngles, rightAngleThreshold)
    lineAngles(lineAngles < 0) = lineAngles(lineAngles < 0) + pi;
    cornerAngle = abs(lineAngles(1) - lineAngles(2));
    if cornerAngle > rightAngleThreshold % Don't consider triangles with sharp angles
        objectLengths(7:end,:) = [];
        indexOffset = 0;
    else %Don't consider triangles with square angles
        objectLengths(5:6,:) = [];
        indexOffset = 2;
    end
    error = sortedLengths - objectLengths';
    match_error = vecnorm(error);
    [min_error, min_index] = min(match_error);
    
    error2 = sortedLengths([2,1]) - objectLengths(5:end,:)';
    match_error2 = vecnorm(error2);
    [min_error2, min_index2] = min(match_error2);
    if min_error2 < min_error
        min_index = min_index2 + 4;
    end
    
    objectNumber = min_index + indexOffset;
end

