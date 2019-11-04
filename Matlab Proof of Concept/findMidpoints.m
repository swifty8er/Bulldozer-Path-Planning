function points = findMidpoints(point1, point2, split_dist)
    %plot line
    %figure(1); hold on;
    %plot([point1(1),point2(1)],[point1(2),point2(2)])
    %determine distance of line
    dist = ptDist(point1, point2);
    %according to the desired distance between extra m,id points and the
    %end points find the number that can be created
    num_of_midpoints = floor(dist/split_dist) - 1;
    if (num_of_midpoints > 0)
        %find the distance of each midpoint from point 1
        int_dist = [dist/(num_of_midpoints+1):dist/(num_of_midpoints+1):dist]';
        %remove the distance value which would give point 2
        if (size(int_dist,1) > num_of_midpoints)
            int_dist(num_of_midpoints+1) = [];
        end
        %find angle between points
        angle = atan2(point2(2)-point1(2),point2(1)-point1(1));
        %find midpoints
        points = point1(1) + int_dist*cos(angle);
        points(:,2) = point1(2) + int_dist*sin(angle);
        %plot midpoints
        %plot(points(:,1),points(:,2), '*r')
    else
        points = [];
    end
return;
end