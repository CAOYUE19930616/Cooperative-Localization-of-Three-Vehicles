function angleA = prepareNormalizedAngleDiff(angleA, angleB)
    if angleA>=angleB
        while norm(angleA-angleB) > pi
            angleA = angleA - 2*pi;
        end
    else
        while norm(angleA-angleB) > pi
            angleA = angleA + 2*pi;
        end
    end
end
