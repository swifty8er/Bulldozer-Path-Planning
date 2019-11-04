%finds the all possible push points a vehicle can reach from its current
%position (push points that have an associated disk on them) 
function [possible_points, point_paths] = findReachablePushPointsVer1(vehicle_pos, disk_pos, CellVG, ...
    VGArray, push_points, PG)
    possible_points = [];
    point_paths = {};
    S = struct('pos', {}, 'path', {});
    %push vehicle pos into the stack
    for r = 1:length(vehicle_pos)
        curr_node.pos = vehicle_pos(r);
        curr_node.path = vehicle_pos(r);
        S(r) = curr_node;
    end
    visited_nodes = zeros(1, length(CellVG) + size(VGArray,2));
    visited_nodes(vehicle_pos) = true;
    visited_nodes(disk_pos) = true;
    %find range nescessary indices to check for possible push points
    check_indices = [];
    for p = 1:length(disk_pos)
        for k = 1:size(push_points,1)
            if(push_points(k,3) == disk_pos(p))
                if (size(check_indices,1) < p)
                    check_indices(p,1) = k;
                    check_indices(p,2) = k;
                else
                    check_indices(p,2) = k;
                end
            end
        end
    end
    [ii, ~] = find(check_indices == 0);
    check_indices(ii, :) = [];
    if (~isempty(check_indices))
        max_num_points = sum(check_indices(:,2)-check_indices(:,1)+1);
    else
        max_num_points = 0;
    end
    %find all possible push points by a depth first search
    while ((~isempty(S)) && (length(possible_points) < max_num_points))
        [curr_node, S] = pop(S);
        new_positions = CellVG{curr_node.pos};
        %add unvisited nodes to queue
        for i = 1:length(new_positions)
            if(visited_nodes(new_positions(i)) == false)
                S = push(S, new_positions(i), curr_node.path);
                visited_nodes(new_positions(i)) = true;
            end
        end
        %add all accessible push points
        for q = 1:size(check_indices,1)
            for j = 0:check_indices(q,2)-check_indices(q,1)
                if ((VGArray(curr_node.pos, check_indices(q,1)+j) ~= 0) && ...
                        (visited_nodes(check_indices(q,1)+j+length(CellVG)) == false) && ...
                        (isempty(find(disk_pos == push_points(check_indices(q,1)+j,6),1))) && ...
                        (PG(push_points(check_indices(q,1)+j,3),push_points(check_indices(q,1)+j,6)) == 1))
                        possible_points = [possible_points, check_indices(q,1)+j+length(CellVG)];
                        point_paths{end+1} = [curr_node.path, check_indices(q,1)+j+length(CellVG)];
                        visited_nodes(check_indices(q,1)+j+length(CellVG)) = true;
                end
            end
        end
    end
return;
end

function [node, S] = pop(S)
    if (~isempty(S))
        node = S(end);
        S(end) = [];
    else
        node = [];
        S = [];
    end
return;
end

function S = push(S, pos, path)
    path(end+1) = pos;
    node.pos = pos;
    node.path = path;
    S(length(S) + 1) = node;
return;
end


function PQ = sortQueue(PQ)
    for i = 1:length(PQ)-1
        for j = 1:length(PQ)-i
            if (PQ(j).cf > PQ(j+1).cf)
                tmp = PQ(j);
                PQ(j) = PQ(j+1);
                PQ(j+1) = tmp;
            end
        end
    end
return;
end