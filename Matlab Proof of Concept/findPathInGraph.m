function path = findPathInGraph(start, goal, nodes, Graph)

    %set up visited table
    visited = inf(size(Graph, 1), 1);


    %A star search to find solution
    %initial state
    goal_node = nodes(goal,:);
    start_node = nodes(start,:);
    curr_node.pos = start;
    curr_node.path = nodes(start,:);
    curr_node.travelled = 0;
    curr_node.cf = sqrt((start_node(1) - goal_node(1))^2 + (start_node(2) - goal_node(2))^2);
    %push into priority queue
    visited(start) = curr_node.cf;
    PQ = curr_node;
    % v = zeros(cs_size(1), cs_size(2)); %%%% ERROR CHECK

    %if the queue is not empty or the destination has not been reached then
    %continue search
    while ((~isempty(PQ)) && (curr_node.pos ~= goal))
        [curr_node, PQ] = pop(PQ);
        for i = 1:size(Graph,1)
            if (Graph(curr_node.pos, i) > 0)
                %check if already been travelled to in the path and check if path is better
                curr_travel = i;
                curr_travel_node = nodes(i,:);
                curr_travelled = curr_node.travelled + 0.5;
                curr_travel_cf = curr_travelled + sqrt((curr_travel_node(1) - goal_node(1))^2 + (curr_travel_node(2) - goal_node(2))^2);
                if (visited(curr_travel) == inf)
                    visited(curr_travel) = curr_travel_cf;
        %             v(curr_travel_node(1), curr_travel_node(2)) = 1; %%%%% ERROR CHECK
                    PQ = push(PQ, curr_travel, curr_travelled, curr_node.path, curr_travel_cf, curr_travel_node);
                elseif (curr_travel_cf < visited(curr_travel))
                    visited(curr_travel) = curr_travel_cf;
        %             v(curr_travel_node(1), curr_travel_node(2)) = 1; %%%%% ERROR CHECK
                    PQ = push(PQ, curr_travel, curr_travelled, curr_node.path, curr_travel_cf, curr_travel_node);
                    %remove point extra point in the queue
                    ii = pointInQueue(curr_travel, PQ);
                    if ((ii ~= 0) && (ii ~= length(PQ)))
                        PQ(ii) = [];
                    end
                end
            end
        end
    %     figure(1) %%%%% ERROR CHECK
    %     imshow(v) %%%%% ERROR CHECK
        PQ = sortQueue(PQ);
    end

    if (curr_node.pos == goal)
        path = curr_node.path;
    else
        path = [];
    end

return;
end

function [node, PQ] = pop(PQ)
    if (~isempty(PQ))
        node = PQ(1);
        PQ(1) = [];
    else
        node = [];
        PQ = [];
    end
return;
end


function PQ = push(PQ, pos, travelled, path, cf, path_node)
    path(end+1,:) = path_node;
    node.pos = pos;
    %node.cf = travelled + sqrt((node.pos(1) - goal(1))^2 + (node.pos(2) - goal(2))^2);
    node.cf = cf;
    node.path = path;
    node.travelled = travelled;
    PQ(length(PQ) + 1) = node;
return;
end

function new_PQ = sortQueue(PQ)
    if (~isempty(PQ))
        cf_list = zeros(1,length(PQ));
        new_PQ(length(PQ)) =  PQ(1);
        for i = 1:length(PQ)
            cf_list(i) = PQ(i).cf;
        end
        [~,ii] = sort(cf_list);
        for i = 1:length(PQ)
            new_PQ(i) = PQ(ii(i));
        end
    else
        new_PQ = [];
    end
return;
end

function index = pointInQueue(pos, PQ)
    index  = 0;
    i = 1;
    while ((i <= length(PQ)) && (index == 0))
        curr_node = PQ(i);
        curr_pos = curr_node.pos;
        if (pos ==  curr_pos)
            index = i;
        end
        i = i + 1;
    end
return;
end


function node = pickNode(nodes, fig_num)
    figure(fig_num);
    point = ginput(1);
    closest_dist = inf;
    %check all nodes
    for i = 1:length(nodes)
        %find that particular node's current position
        curr_node = nodes(i,:);
        %find the distance between the chosen point and the node
        curr_dist = sqrt((point(1) - curr_node(1))^2+(point(2) - curr_node(2))^2);
        if (curr_dist < closest_dist)
            closest_dist = curr_dist;
            node = i;
        end
    end
return;
end