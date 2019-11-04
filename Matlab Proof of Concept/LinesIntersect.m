function intersect = LinesIntersect(Line1, Line2)
    x1 = Line1.Point1_x;
    x2 = Line1.Point2_x;
    x3 = Line2.Point1_x;
    x4 = Line2.Point2_x;
    y1 = Line1.Point1_y;
    y2 = Line1.Point2_y;
    y3 = Line2.Point1_y;
    y4 = Line2.Point2_y;
    diff_x = x1 - x3;
    diff_y = y1 - y3;
    denominator = (x4 - x3).*(y1 - y2) - (x1 - x2).*(y4 - y3);
    if (denominator ~= 0)
        t1 = ((y3 - y4).*diff_x + (x4 - x3).*diff_y)/denominator;
        t2 = ((y1 - y2).*diff_x + (x2 - x1).*diff_y)/denominator;
        if ((t1 <= 1) && (t1 >= 0) && (t2 <= 1) && (t2 >= 0))
            intersect = true;
        else
            intersect = false;
        end
    else
        intersect = false;
    end    
return;
end