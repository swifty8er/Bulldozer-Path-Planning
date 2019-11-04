%find the perpendicuar distance between a point and a line if within the
%bounds of the line (line is not extended) otherwise find the distance to the closest point
function dist = point2LineDist(line, point)
    perp_gradient = -(line.Point1_x-line.Point2_x)/(line.Point1_y-line.Point2_y);
    if (isinf(perp_gradient))
        inbounds = ((point(1) <= max([line.Point1_x,line.Point2_x])) && (point(1) >= min([line.Point1_x,line.Point2_x])));
    else
    perp_intercept_1 = line.Point1_y - perp_gradient*line.Point1_x;
    perp_intercept_2 = line.Point2_y - perp_gradient*line.Point2_x;
    perp_lines_y(1) = perp_gradient*point(1) + perp_intercept_1;
    perp_lines_y(2) = perp_gradient*point(1) + perp_intercept_2;
    inbounds = ((point(2) <= max(perp_lines_y)) && (point(2) >= min(perp_lines_y)));
    end
    if (inbounds)
        a = [line.Point1_x,line.Point1_y,0] - [line.Point2_x,line.Point2_y,0];
        b = [point,0] - [line.Point2_x,line.Point2_y,0];
        dist = norm(cross(a,b)) / norm(a);
    else
        distances(1) = sqrt((point(1)-line.Point1_x)^2+(point(2)-line.Point1_y)^2);
        distances(2) = sqrt((point(1)-line.Point2_x)^2+(point(2)-line.Point2_y)^2);
        dist = min(distances);
    end
return;
end