function poses = findValidTravelPoints(pos, CS, min_x, min_y, max_x, max_y)
    %check the four possible travel directions north, south east and west
    pos_dir = [-1,0;1,0;0,1;0,-1]; %in the order N, S ,E W
    %for all new travel directions
    poses = [];
    for i = 1:size(pos_dir,1)
        new_pos = pos + pos_dir(i,:);
        if ((new_pos(2) >= min_x) && (new_pos(2) <= max_x) && ...
                (new_pos(1) >= min_y) && (new_pos(1) <= max_y) && ...
                (CS(new_pos(1),new_pos(2)) == 0))
            poses = [poses; new_pos];
        end
    end
return;
end