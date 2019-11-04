%find if it is possible to push between nodes
function push_point = pushPoint(from, dest, Map, from_id, dest_id)
    pushable = true;
    push_point = [];
    %add the boundary to the Map obstacles
    Map.Obstacles{end+1} = Map.boundary;
    %initialise vehicle circle
    angle = atan2(from(2) - dest(2), from(1) - dest(1));
    centre(1) = from(1) + (Map.disc_radius+Map.vehicle_radius)*cos(angle);
    centre(2) = from(2) + (Map.disc_radius+Map.vehicle_radius)*sin(angle);
%     Line1.Point1_x = from(1);
%     Line1.Point1_y = from(2);
%     Line1.Point2_x = from(1) + (Map.disc_radius+Map.vehicle_radius*2)*cos(angle);
%     Line1.Point2_y = from(2) + (Map.disc_radius+Map.vehicle_radius*2)*sin(angle);
    i = 1;
    while ((i <= length(Map.Obstacles)) && (pushable == true))
        j = 1;
        curr_obs = cell2mat(Map.Obstacles(i));
        while ((j < length(curr_obs)) && (pushable == true))
            Line.Point1_x = curr_obs(j,1);
            Line.Point1_y = curr_obs(j,2);
            Line.Point2_x = curr_obs(j+1,1);
            Line.Point2_y = curr_obs(j+1,2);
            pushable = ~CircleAndLineIntersect(centre, Map.vehicle_radius, Line);
            j = j + 1;
        end
        i = i + 1;
    end
    if (pushable == true)
        push_point(1) = from(1) + (Map.disc_radius+Map.vehicle_radius)*cos(angle);
        push_point(2) = from(2) + (Map.disc_radius+Map.vehicle_radius)*sin(angle);
        push_point(3) = from_id; %start id
        push_point(4) = dest(1) + (Map.disc_radius+Map.vehicle_radius)*cos(angle);
        push_point(5) = dest(2) + (Map.disc_radius+Map.vehicle_radius)*sin(angle);
        push_point(6) = dest_id; %destination id
    end
return;
end