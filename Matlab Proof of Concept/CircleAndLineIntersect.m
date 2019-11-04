%find if a circle and a line intersect
function intersect = CircleAndLineIntersect(centre, radius, line)
    dist = point2LineDist(line, centre);
    if (dist <= radius)
        intersect = true;
    else
        intersect = false;
    end
end