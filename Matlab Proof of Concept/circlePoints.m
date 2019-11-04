function circle = circlePoints(centre, radius, numOfPoints)
    circle = zeros(numOfPoints,2);
    for i = 1:numOfPoints
        circle(i,1) = centre(1) + radius*cos((i-1)*(2*pi/(numOfPoints-1)));
        circle(i,2) = centre(2) + radius*sin((i-1)*(2*pi/(numOfPoints-1)));
    end
return;
end