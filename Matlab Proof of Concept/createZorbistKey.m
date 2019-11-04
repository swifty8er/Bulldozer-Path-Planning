function Z = createZorbistKey(num_of_nodes, num_of_bits)
    threshold = 2^num_of_bits - 1;
    %unique may take too long
    
    %find a unique number for vehicle at first node
    node_num_id.vehicle = randi(threshold);
    %find a unique number for disk at first node
    node_num_id.disk = randi(threshold);
    while (node_num_id.disk == node_num_id.vehicle)
        node_num_id.disk = randi(threshold);
    end
    %find a unique number for empty at first node
    node_num_id.empty = randi(threshold);
    while ((node_num_id.empty == node_num_id.vehicle) || (node_num_id.empty == node_num_id.disk))
        node_num_id.empty = randi(threshold);
    end
    % store in Zorbist Key
    Map_num_ids(1) = node_num_id;
    % keep track of all current numbers
    rand_nums = [node_num_id.vehicle, node_num_id.disk, node_num_id.empty];
    
    for i = 2:num_of_nodes
        %find a unique number for vehicle at firt node
        node_num_id.vehicle = randi(threshold);
        while (~isempty(find(rand_nums == node_num_id.vehicle,1)))
            node_num_id.vehicle = randi(threshold);
        end
        rand_nums(end+1) = node_num_id.vehicle;
        %find a unique number for disk at firt node
        node_num_id.disk = randi(threshold);
        while (~isempty(find(rand_nums == node_num_id.disk,1)))
            node_num_id.disk = randi(threshold);
        end
        rand_nums(end+1) = node_num_id.disk;
        %find a unique number for vehicle at firt node
        node_num_id.empty = randi(threshold);
        while (~isempty(find(rand_nums == node_num_id.empty,1)))
            node_num_id.empty = randi(threshold);
        end
        rand_nums(end+1) = node_num_id.empty;
        Map_num_ids(i) = node_num_id;
    end
    
    Z = Map_num_ids;
return;
end