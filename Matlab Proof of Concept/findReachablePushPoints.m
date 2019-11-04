%finds the all possible push points a vehicle can reach from its current
%position (push points that have an associated disk on them) 
function [possible_points, point_paths] = findReachablePushPoints(vehicle_pos, disk_pos, CellVG, ...
    VGArray, push_points, PG, CellPG, NodesBlockVertices, goal_pos, nodes, tolerance)
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
                        %get a list of the disks that aren't being moved
                        other_disks = disk_pos;
                        other_disks(kk) = [];
                        %check to see if there are any pushable connections
                        %from the destination node
                        filter_CellPG = filterCellPG(CellPG, NodesBlockVertices, other_disks, disk_pushed_to);
                        %pushable connections have been found
                        %find push points
                        disk_pushed_from = disk_pushed_to;
                        disks_pushed_to = filter_CellPG{disk_pushed_from};
                        valid_push_points = findPushPoints(disk_pushed_from, disks_pushed_to, push_points);
                        %find if these push points are reachable with
                        %new disk placement
                        u = 1;
                        valid = false;
                        %search through all push points to find one that is valid
                        while ((u <= length(valid_push_points)) && (valid == false))
                            v = 1;
                            disk_is_blocking = false;
                            %check whether there are any disks that are
                            %too close and blocking the push points
                            while ((v <= length(other_disks)) && (disk_is_blocking == false))
                                curr_dist = ptDist(nodes(other_disks(v),:), push_points(valid_push_points(u),1:2));
                                if ((tolerance-curr_dist) > eps(100))
                                    disk_is_blocking = true;
                                end
                                v = v + 1;
                            end
                            if (disk_is_blocking == false)
                                valid = true;
                            end
                            u = u + 1;
                        end
                        if (valid == true)
                            possible_points = [possible_points, check_indices(q,1)+j+length(CellVG)];
                            point_paths{end+1} = [curr_node.path, check_indices(q,1)+j+length(CellVG)];
                            visited_nodes(check_indices(q,1)+j+length(CellVG)) = true;
                        else
                            %find the push points that were neglected
                            %from the CellPG
                            nodes_pushed_to = CellPG{disk_pushed_from};
                            valid_push_points = findPushPoints(disk_pushed_from, nodes_pushed_to, push_points);
                            %list of disks that are required to move to make a certain push point valid
                            disk_req_move = findDiskThatNeedToMove(NodesBlockVertices, other_disks, disk_pushed_from, nodes_pushed_to, length(CellPG));
                            %find if these push points are reachable with
                            %new disk placement
                            for u = 1:length(valid_push_points)
                                for v = 1:length(other_disks)
                                    curr_dist = ptDist(nodes(other_disks(v),:), push_points(valid_push_points(u),1:2));
                                    if ((tolerance-curr_dist) > eps(100))
                                        if (isempty(find(disk_req_move{u} == other_disks(v),1)))
                                            disk_req_move{u}(end+1) = other_disks(v);
                                        end
                                    end
                                end
                            end
                            %order all push points to be investigated
                            %by the number of disks that they require
                            %to be moved from least to most
                            num_of_disk_list = zeros(1,length(disk_req_move));
                            sorted_disk_req_move = {};
                            sorted_disk_req_move{length(disk_req_move)} =  [];
                            for i = 1:length(disk_req_move)
                                num_of_disk_list(i) = length(disk_req_move{i});
                            end
                            [~,ii] = sort(num_of_disk_list);
                            for i = 1:length(disk_req_move)
                                sorted_disk_req_move{i} = disk_req_move{ii(i)};
                            end
                            %Now take the first possible push point and
                            %check that the disks that are in the way
                            %can be moved
                            i = 1;
                            valid = false;
                            filter_CellPG = filterCellPG(CellPG, NodesBlockVertices, disk_pushed_from, other_disks);
                            %search through all possible push points to
                            %see if any are valid
                            while ((i <= length(sorted_disk_req_move)) && (valid == false))
                                u = 1;
                                valid_pp = true;
                                %search through all the disks that need
                                %moving to see if any have valid push
                                %decisions
                                while ((u <=length(sorted_disk_req_move{i})) && (valid_pp == true))
                                    if (~isempty(filter_CellPG{sorted_disk_req_move{i}(u)}))
                                        v = 1;
                                        valid_disk_move = false;
                                        %find the push points for the
                                        %disk that needs moving
                                        pps = findPushPoints(sorted_disk_req_move{i}(u), filter_CellPG{sorted_disk_req_move{i}(u)}, push_points);
                                        %check that at least on push
                                        %point isn't too close to the
                                        %new disk location
                                        while ((v <= length(pps)) && (valid_disk_move == false))
                                            curr_pp = push_points(pps(v),1:2);
                                            curr_dist = ptDist(nodes(disk_pushed_from,:), curr_pp);
                                            if ((tolerance-curr_dist) <= eps(100))
                                                valid_disk_move = true;
                                            end
                                            v = v + 1;
                                        end
                                        if (valid_disk_move == false)
                                            valid_pp = false;
                                        end
                                    else
                                        valid_pp = false;
                                    end
                                    u = u + 1;
                                end
                                if (valid_pp == true)
                                    valid = true;
                                end
                                i = i + 1;
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

%find the push points given a from disk and disks that you are travelling
%to. Assume the push point list is sorted
function valid_push_points = findPushPoints(disk_from, disks_to, push_points)
    if (~isempty(disks_to))
        k = 1;
        found = false;
        finished = false;
        valid_push_points = [];
        while ((k <= size(push_points,1)) && (finished == false))
            if(push_points(k,3) == disk_from)
                kk = 1;
                while ((kk <= length(disks_to)) && (length(valid_push_points) < length(disks_to)))
                    if (push_points(k,6) == disks_to(kk))
                        valid_push_points = [valid_push_points, k];
                    end
                    kk = kk + 1;
                end
            elseif (found == true)
                finished = true;
            end
            k = k + 1;
        end
    else
        valid_push_points = [];
    end
return;
end

function new_CellPG = filterCellPG(CellPG, NodesBlockVertices, filter_disks, disks_of_interest)
    %check to see if there are any pushable connections
    %from the disks of interest node
    new_CellPG = CellPG;
    for i = 1:length(disks_of_interest)
        p = 1;
        while ((p <= length(filter_disks)) && (~isempty(new_CellPG{disks_of_interest(i)})))
            r = filter_disks(p);
            kk = find(new_CellPG{disks_of_interest(i)} == r,1);
            new_CellPG{disks_of_interest(i)}(kk) = [];
            u = 1;
            while ((u <= size(NodesBlockVertices(r).vertices,1)) && ...
                    (NodesBlockVertices(r).vertices(u,1) <= disks_of_interest(i)) && ...
                    (~isempty(new_CellPG{disks_of_interest(i)})))
                ii = NodesBlockVertices(r).vertices(u,1);
                jj = NodesBlockVertices(r).vertices(u,2);
                if (((ii == disks_of_interest(i)) || (jj == disks_of_interest(i))) && (ii <= length(CellPG)) && (jj <= length(CellPG)))
                    if (ii == disks_of_interest(i))
                        kk = find(new_CellPG{ii} == jj,1);
                        new_CellPG{ii}(kk) = [];
                    else
                        kk = find(new_CellPG{jj} == ii,1);
                        new_CellPG{jj}(kk) = [];
                    end
                end
                u = u + 1;
            end
            p = p + 1;
        end
    end
return;
end

function disk_req_move = findDiskThatNeedToMove(NodesBlockVertices, filter_disks, disk_of_interest, nodes_to, num_of_nodes)
    %check to see if there are any pushable connections
    %from the disks of interest node
    disk_req_move = {};
    disk_req_move{length(nodes_to)} = [];
    for p = 1:length(filter_disks)
        r = filter_disks(p);
        kk = find(nodes_to == r,1);
        if (~isempty(kk))
            disk_req_move{kk}(end+1) = r;
        end
        u = 1;
        while ((u <= size(NodesBlockVertices(r).vertices,1)) && ...
                (NodesBlockVertices(r).vertices(u,1) <= disk_of_interest))
            ii = NodesBlockVertices(r).vertices(u,1);
            jj = NodesBlockVertices(r).vertices(u,2);
            if (((ii == disk_of_interest) || (jj == disk_of_interest)) && (ii <= num_of_nodes) && (jj <= num_of_nodes))
                if (ii == disk_of_interest)
                    kk = find(nodes_to == jj,1);
                    if (~isempty(kk))
                        disk_req_move{kk}(end+1) = r;
                    end
                else
                    kk = find(nodes_to == ii,1);
                    if (~isempty(kk))
                        disk_req_move{kk}(end+1) = r;
                    end
                end
            end
            u = u + 1;
        end
    end
return;
end