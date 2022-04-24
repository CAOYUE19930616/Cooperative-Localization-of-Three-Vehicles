function posDisplayed = displayPos(pos, length, style)
    if size(pos, 1)==2
        plot(pos(1), pos(2), style);
    else
        if(isa(style, 'char'))
            posDisplayed = quiver(pos(1,:), pos(2,:), length*cos(pos(3,:)), length*sin(pos(3,:)), style, 'LineWidth', 2);
        else
            posDisplayed = quiver(pos(1,:), pos(2,:), length*cos(pos(3,:)), length*sin(pos(3,:)), 'color', style);
        end
    end
end

