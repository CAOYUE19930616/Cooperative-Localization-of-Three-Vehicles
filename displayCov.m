function displayCov(pos, cov, proba, style)
    ellipse(pos(1:2), cov(1:2,1:2), proba, style);
    if size(pos, 1)>2
        displayCone(pos, cov(3,3), proba, 1, style);
    end
end

