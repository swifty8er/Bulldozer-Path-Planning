function CS = Map2CS(Map)
    
    % set up CS
    CS = ones((Map.max_y-Map.min_y)/Map.grid_size,(Map.max_x-Map.min_x)/Map.grid_size);
    
    %add Boundary
    num_of_lines = size(Map.boundary,1) - 1;
    curr_obs = Map.boundary;
    for j = 1:num_of_lines
        point1 = curr_obs(j,:);
        point2 = curr_obs(j+1,:);
        %fill in the line on the CS space
        index1 = point2index(point1(1),point1(2), Map.grid_size, Map.min_x, Map.max_x, Map.min_y, Map.max_y);
        index2 = point2index(point2(1),point2(2), Map.grid_size, Map.min_x, Map.max_x, Map.min_y, Map.max_y);
        CS = fillInLine(CS, index1, index2, 0);
    end
    %fill in the completed obstacle
    %Get a starting point
    %used to specify possible directions of traversal
    addable = [1,-1,0,0,1,-1,1,-1; 0,0,1,-1,1,-1,-1,1];
    k = 1;
    grid_obs = point2index(curr_obs(:,1),curr_obs(:,2), Map.grid_size, Map.min_x, Map.max_x, Map.min_y, Map.max_y);
    in_obstacle = false;
    while ((k <=size(addable,2)) && (in_obstacle == false))
        node = [index2(1)+addable(1,k), index2(2)+addable(2,k)];
        [in_obstacle, on_obstacle] = inpolygon(node(1),node(2),grid_obs(:,1),grid_obs(:,2)); % this line needs to be looked at
        if ((in_obstacle == true) && (on_obstacle == true))
            in_obstacle = false;
        end
        k = k + 1;
    end
    Q = node;
    CS(node(1),node(2)) = 1;
    %Do a breadth first search only search NSEW
    while (~isempty(Q))
        [curr_node, Q] = pop(Q);
        %find which adjacent nodes are valid
        addable = [1, -1, 0, 0; 0,0,1,-1];
        for k = 1:size(addable,2)
            new_node = [curr_node(1)+addable(1,k), curr_node(2)+addable(2,k)];
            if (ifIndexIsValid(new_node, size(CS,1), size(CS,2)))
                if (CS(new_node(1),new_node(2)) ~= 0)
                    % add them to the queue
                    Q = push(new_node, Q);
                    CS(new_node(1),new_node(2)) = 0;
                end
            end
        end            
    end 

    % Go through all every obstacle
    for i = 1:length(Map.Obstacles)
        num_of_lines = size(Map.Obstacles{i},1) - 1;
        curr_obs = Map.Obstacles{i};
        for j = 1:num_of_lines
            point1 = curr_obs(j,:);
            point2 = curr_obs(j+1,:);
            %fill in the line on the CS space
            index1 = point2index(point1(1),point1(2), Map.grid_size, Map.min_x, Map.max_x, Map.min_y, Map.max_y);
            index2 = point2index(point2(1),point2(2), Map.grid_size, Map.min_x, Map.max_x, Map.min_y, Map.max_y);
            CS = fillInLine(CS, index1, index2, 1);
        end
        %fill in the completed obstacle
        %Get a starting point
        %used to specify possible directions of traversal
        addable = [1,-1,0,0,1,-1,1,-1; 0,0,1,-1,1,-1,-1,1];
        k = 1;
        grid_obs = point2index(curr_obs(:,1),curr_obs(:,2), Map.grid_size, Map.min_x, Map.max_x, Map.min_y, Map.max_y);
        in_obstacle = false;
        while ((k <=size(addable,2)) && (in_obstacle == false))
            node = [index2(1)+addable(1,k), index2(2)+addable(2,k)];
            [in_obstacle, on_obstacle] = inpolygon(node(1),node(2),grid_obs(:,1),grid_obs(:,2)); % this line needs to be looked at
            if ((in_obstacle == true) && (on_obstacle == true))
                in_obstacle = false;
            end
            k = k + 1;
        end
        Q = node;
        CS(node(1),node(2)) = 1;
        %Do a breadth first search only search NSEW
        while (~isempty(Q))
            [curr_node, Q] = pop(Q);
            %find which adjacent nodes are valid
            addable = [1, -1, 0, 0; 0,0,1,-1];
            for k = 1:size(addable,2)
                new_node = [curr_node(1)+addable(1,k), curr_node(2)+addable(2,k)];
                if (CS(new_node(1),new_node(2)) ~= 1)
                    % add them to the queue
                    Q = push(new_node, Q);
                    CS(new_node(1),new_node(2)) = 1;
                end
            end            
        end 
    end

return;
end

function index = point2index(x,y, grid_size, min_x, max_x, min_y, max_y)
    adj_x = min(max_x-grid_size*0.01, max(min_x, x));
    adj_y = min(max_y-grid_size*0.01, max(min_y, max_y - y));
    index(:,1) = floor(adj_y/grid_size) + 1;
    index(:,2) = floor(adj_x/grid_size) + 1;
return;
end

function valid = ifIndexIsValid(index, max_i, max_j)
    if ((index(1) <= 0) || (index(2) <= 0) || ...
            (index(1) > max_i) || (index(2) > max_j))
        valid = false;
    else
        valid = true;
    end
return;
end

function [node, Q] = pop(Q)
    if (~isempty(Q))
        node = Q(1,:);
        Q(1,:) = [];
    else
        node = [];
        Q = [];
    end
return;
end

function Q = push(node, Q)
    Q(size(Q,1) + 1,:) = node;
return;
end