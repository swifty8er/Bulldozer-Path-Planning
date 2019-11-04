%finds the all possible push points a vehicle can reach from its current
%position (push points that have an associated disk on them) 
function [possible_points, point_paths] = findReachablePushPoints(vehicle_pos, disk_pos, CellVG, ...
    VGArray, push_points, PG, CellPG, NodesBlockVertices, goal_pos, CellPushPointVG)
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
                    if (isempty(find(goal_pos == push_points(check_indices(q,1)+j,6),1)))
                        %check if being pushed into a dead-end
                        disk_pushed_from = push_points(check_indices(q,1)+j,3);
                        disk_pushed_to = push_points(check_indices(q,1)+j,6);
                        kk = find(disk_pos == disk_pushed_from,1);
                        %get a least of the disks that aren't being moved
                        other_disks = disk_pos;
                        other_disks(kk) = [];
                        %check to see if there are any pushable connections
                        %from the destination node
                        p = 1;
                        while ((p <= length(other_disks)) && (~isempty(CellPG{disk_pushed_to})))
                            r = other_disks(p);
                            kk = find(CellPG{disk_pushed_to} == r,1);
                            CellPG{disk_pushed_to}(kk) = [];
                            u = 1;
                            while ((u <= size(NodesBlockVertices(r).vertices,1)) && ...
                                    (NodesBlockVertices(r).vertices(u,1) <= disk_pushed_to) && ...
                                    (~isempty(CellPG{disk_pushed_to})))
                                ii = NodesBlockVertices(r).vertices(u,1);
                                jj = NodesBlockVertices(r).vertices(u,2);
                                if (((ii == disk_pushed_to) || (jj == disk_pushed_to)) && (ii <= size(PG,1)) && (jj <= size(PG,1)))
                                    if (ii == disk_pushed_to)
                                        kk = find(CellPG{ii} == jj,1);
                                        CellPG{ii}(kk) = [];
                                    else
                                        kk = find(CellPG{jj} == ii,1);
                                        CellPG{jj}(kk) = [];
                                    end
                                end
                                u = u + 1;
                            end
                            p = p + 1;
                        end
                        if (~isempty(CellPG{disk_pushed_to}))
                            %pushable connections have been found
                            %find push points
                            k = 1;
                            found = false;
                            finished = false;
                            disk_pushed_from = disk_pushed_to;
                            disks_pushed_to = CellPG{disk_pushed_from};
                            valid_push_points = [];
                            while ((k <= size(push_points,1)) && (finished == false))
                                if(push_points(k,3) == disk_pushed_from)
                                    kk = 1;
                                    while ((kk <= length(disks_pushed_to)) && (length(valid_push_points) < length(disks_pushed_to)))
                                        if (push_points(k,6) == disks_pushed_to(kk))
                                            valid_push_points = [valid_push_points, k];
                                        end
                                        kk = kk + 1;
                                    end
                                elseif (found == true)
                                    finished = true;
                                end
                                k = k + 1;
                            end
                            %find if these push points are reachable with
                            %new disk placement
                            v = 1;
                            valid = false;
                            while ((v <= length(valid_push_points)) && (valid == false))
                                curr_pp = valid_push_points(v);
                                curr_pp_wrt_nodes = curr_pp + length(CellVG);
                                p = 1;
                                while ((p <= length(other_disks)) && (~isempty(CellPushPointVG{curr_pp})))
                                    r = other_disks(p);
                                    kk = find(CellPushPointVG{curr_pp} == r,1);
                                    CellPushPointVG{curr_pp}(kk) = [];
                                    u = 1;
                                    while ((u <= size(NodesBlockVertices(r).vertices,1)) && ...
                                            (NodesBlockVertices(r).vertices(u,1) <= curr_pp_wrt_nodes) && ...
                                            (~isempty(CellPushPointVG{curr_pp})))
                                        ii = NodesBlockVertices(r).vertices(u,1);
                                        jj = NodesBlockVertices(r).vertices(u,2);
                                        if (((ii == curr_pp_wrt_nodes)  && (jj <= size(PG,1))) || ((jj == curr_pp_wrt_nodes) && (ii <= size(PG,1))))
                                            if (ii == curr_pp_wrt_nodes)
                                                kk = find(CellPushPointVG{ii - length(CellVG)} == jj,1);
                                                CellPushPointVG{ii - length(CellVG)}(kk) = [];
                                            else
                                                kk = find(CellPushPointVG{jj - length(CellVG)} == ii,1);
                                                CellPushPointVG{jj - length(CellVG)}(kk) = [];
                                            end
                                        end
                                        u = u + 1;
                                    end
                                    p = p + 1;
                                end
                                if (~isempty(CellPushPointVG{valid_push_points(v)}))
                                    valid = true;
                                end
                                v = v + 1;
                            end
                            if (valid == true)
                                possible_points = [possible_points, check_indices(q,1)+j+length(CellVG)];
                                point_paths{end+1} = [curr_node.path, check_indices(q,1)+j+length(CellVG)];
                                visited_nodes(check_indices(q,1)+j+length(CellVG)) = true;
                            end
                        end
                    else
                        possible_points = [possible_points, check_indices(q,1)+j+length(CellVG)];
                        point_paths{end+1} = [curr_node.path, check_indices(q,1)+j+length(CellVG)];
                        visited_nodes(check_indices(q,1)+j+length(CellVG)) = true;
                    end
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