%Marcus Swift
%Program for solving a simple bulldozer problem
%boundaries 7m by 7m

% IMPROVEMENTS AND QUESTIONS
% Shrink the memory usage
% Better method for detecting collision with polygon
% Better method for finding collisions for pushability and visibility graph
% Hash table use less memory by using a hash function to turn key into index
% Using RRT to recalculate algorithm's graphs after every move
% Using a probabilistic map to find the nodes required to find a solution
% What is more efficient RRT or precalculated graphs
% expanding the obstacles when calculating visibility graph
% what is more efficient disallowing any edges to pass too close to a node or
% obstacle corner (obstacle corners can be removed from testing once
% obstacles are expanded when calculating visibility graph) or checking if
% vehicle passes through disk on during its path

%create test map
Maps = CreateTestMaps();
for test_map_number = 5 %[63,36,52,53,54,60,61,62,65,70,74,75,77,78,80,83,86,87,89,90,91,93,95,97,98,99]
clearvars -except test_map_number Maps
close all
disp(test_map_number)
Map = Maps{test_map_number};
%plot test map
%plotMap(Map,1);
%Create configuration space
CS = Map2CS(Map);
%create RRM
%nodes = medialAxisPRM(CS, Map);
nodes = findPerfectNodes(Map);
%add vehicle, goal and disks as nodes
point = IsPointInArray(nodes, Map.initial_vehicle_pos_xy);
if (point ~= 0)
     Map.initial_vehicle_pos = point;
else
    nodes(end+1,:) = Map.initial_vehicle_pos_xy;
    Map.initial_vehicle_pos = size(nodes,1);
end
Map.initial_disk_pos = [];
Map.goal_pos = [];
for i = 1:size(Map.initial_disk_pos_xy,1)
    point = IsPointInArray(nodes, Map.initial_disk_pos_xy(i,:));
    if (point ~= 0)
        Map.initial_disk_pos(end+1) =  point;
    else
        nodes(end+1,:) = Map.initial_disk_pos_xy(i,:);
        Map.initial_disk_pos(end+1) = size(nodes,1);
    end
    point = IsPointInArray(nodes, Map.goal_pos_xy(i,:));
    if (point ~= 0)
        Map.goal_pos(end+1) = point;
    else
        nodes(end+1,:) = Map.goal_pos_xy(i,:);
        Map.goal_pos(end+1) = size(nodes,1);
    end
end

plotMap(Map,1);
for i = 1:length(nodes)
    plot(nodes(i,1),nodes(i,2), 'r*','MarkerSize',5);
    text(nodes(i,1),nodes(i,2),num2str(i), 'FontSize', 14);
end

[VG, Cond_Vertices] = createVisibilityGraph(nodes, Map);
plotVisibilityGraph(VG, nodes, 'b', 'c', 2);

%find a list of pushable points and a pushabilitiy graph
node_VG = VG;
[PG, all_push_points] = createPushabilityGraph(VG, nodes, Map);
all_push_points = sortArray(all_push_points, 3);

%split the different types of push points
dest_points = all_push_points(:,4:6);
push_points = all_push_points(:,1:3);
%plot all pushibility points
plotPushabilityGraph(PG, nodes, 'r', 'm' , 1);
for i = 1:size(all_push_points,1)
    plot(dest_points(i,1), dest_points(i,2), 'k*');
    %text(dest_points(i,1),dest_points(i,2),num2str(i+length(nodes)+length(push_points)));
    plot(push_points(i,1), push_points(i,2), 'g*');
    %text(push_points(i,1),push_points(i,2),num2str(i+length(nodes)));
end
for i = 1:length(nodes)
    plot(nodes(i,1),nodes(i,2), 'r*','MarkerSize',5);
    text(nodes(i,1),nodes(i,2),num2str(i), 'FontSize', 14);
end

%add pushibility points to the Visibility Graph then plot the edges
[VG, Cond_Vertices] = addPushPoints(VG, [push_points; dest_points], nodes, Map, Cond_Vertices);
points_and_nodes = [nodes; push_points(:,1:2); dest_points(:,1:2)];
%plotVisibilityGraph(VG, points_and_nodes, 'y', 'k' , 0.25);

%convert VG into cell array for the original nodes
CellNodeVG = VGtoCellArray(node_VG);
%get only the second part of the VG that looks at edge connections between
%nodes and push points
PushPointsVGArray = VG(1:length(nodes), length(nodes)+1:end);

%convert the conditional vertices array of nist of nodes for every vertex
%to a list of vertices for every node position.
NodesBlockVertices = convertToNodesBlockingVertices(Cond_Vertices, length(nodes));

%% Run Algorithm

%set up transposition table
T = createTranspositionTable(length(points_and_nodes));

%A star search to find solution

%initial state
curr_node.vehicle_pos = Map.initial_vehicle_pos;
curr_node.disk_pos = Map.initial_disk_pos;
curr_node.vehicle_path = Map.initial_vehicle_pos;
for i = 1:length(curr_node.disk_pos)
    curr_node.disk_path{i} = Map.initial_disk_pos(i);
end
curr_node.cf = 0;
%calculate cost function

goal_pos = points_and_nodes(Map.goal_pos,:);
%push into priority queue
[T, status] = addToTranspositionTable(curr_node, T);
PQ = createPQ(curr_node, 31, 1);

%set up the master Visibility Graph and master Pushability Graph
master_VG = VG;
master_PG = PG;
master_CellNodeVG = CellNodeVG;
master_PushPointsVGArray = PushPointsVGArray;

%% A Star Section
%if the queue is not empty or the destination has not been reached then
%continue search
%(curr_node.disk_pos(1) == 20) && (curr_node.disk_pos(2) == 13) &&  (curr_node.disk_pos(3) == 16) && (curr_node.disk_pos(4) == 10) && (length(curr_node.disk_path{1,2}) > 1) && (curr_node.disk_path{1,2}(2) == 13)
while ((~isQueueEmpty(PQ)) && (sum(sort(curr_node.disk_pos) ~= sort(Map.goal_pos))))
    [curr_node, PQ] = pop(PQ);
    %Find the correct VG, PG to use by blocking paths that are inaccesible
    %due to disk placement
    for p = 1:length(curr_node.disk_pos)
        r = curr_node.disk_pos(p);
        %this node has now been visited
        if (curr_node.vehicle_pos > length(nodes) + length(push_points))
            test_node.vehicle_pos = push_points(curr_node.vehicle_pos - size(nodes,1) - length(push_points),3);
        elseif (curr_node.vehicle_pos > length(nodes))
            test_node.vehicle_pos = push_points(curr_node.vehicle_pos - size(nodes,1),3);
        else
            test_node.vehicle_pos = curr_node.vehicle_pos;
        end
        test_node.disk_pos = curr_node.disk_pos;
        test_node.cf = curr_node.cf;
        key = getZorbistKey(test_node, T.Zorbist);
        index = mod(key,T.size)+1;
        j = 1;
        found = false;
        while ((j <= length(T.Table{index})) && (found == false))
            if (key == T.Table{index}(j).key)
                found = true;
                T.Table{index}(j).visited = true;
            end
            j = j + 1;
        end
        %now find the updated VG and PG
        for q = 1:size(NodesBlockVertices(r).vertices,1)
            ii = NodesBlockVertices(r).vertices(q,1);
            jj = NodesBlockVertices(r).vertices(q,2);
            VG(ii, jj) = 0;
            VG(jj, ii) = 0;
            if ((ii <= length(nodes)) && (jj <= length(nodes)))
                PG(ii, jj) = 0;
                PG(jj, ii) = 0;
            end
            if (ii <= length(nodes))
                if (jj - length(nodes) > 0)
                    PushPointsVGArray(ii,jj - length(nodes)) = 0;
                end
                kk = find(CellNodeVG{ii} == jj);
                CellNodeVG{ii}(kk) = [];
            end
            if (jj <= length(nodes))
                if (ii - length(nodes) > 0)
                    PushPointsVGArray(jj,ii - length(nodes)) = 0;
                end
                kk = find(CellNodeVG{jj} == ii);
                CellNodeVG{jj}(kk) = [];
            end
        end
        %Now all connections related to that disk node
        VG(r, :) = 0;
        VG(:, r) = 0;
        %PG(r, :) = 0;
        %PG(:, r) = 0;
        PushPointsVGArray(r,:) = 0;
        CellNodeVG{r} = [];
        for s = 1:length(CellNodeVG)
            kk = find(CellNodeVG{s} == r);
            CellNodeVG{s}(kk) = [];
        end
    end
    curr_vehicle_poses = [];
    %find possible push points reachable by the vehicle
    if (curr_node.vehicle_pos > length(nodes))
        for i = 1:length(nodes)
            if (VG(curr_node.vehicle_pos,i) ~= 0)
                curr_vehicle_poses(end+1) = i;
            end
        end
    else
        curr_vehicle_poses = curr_node.vehicle_pos;
    end
    [possible_push_points, possible_push_point_paths] = ...
        findReachablePushPoints(curr_vehicle_poses, curr_node.disk_pos, CellNodeVG, PushPointsVGArray, all_push_points, PG);
    for i = 1:length(possible_push_points)
        %check if already been travelled to in the path and check if path is better
        vehicle_pos = possible_push_points(i) + size(push_points,1);
        disk_num = find(curr_node.disk_pos == push_points(possible_push_points(i) - size(nodes,1),3));
        disk_pos = curr_node.disk_pos;
        disk_pos(disk_num) = dest_points(possible_push_points(i) - size(nodes,1),3);
        dest_pos = points_and_nodes(disk_pos,:);
        start_pos = points_and_nodes(curr_node.disk_pos,:);
        cf = sqrt((dest_pos(disk_num,1)-start_pos(disk_num,1))^2+(dest_pos(disk_num,2)-start_pos(disk_num,2))^2)+curr_node.cf;
        
        %Check Transposition Table
        %find relevant vehicle positions
        test_node.vehicle_pos = push_points(possible_push_points(i) - size(nodes,1),3);
        test_node.disk_pos = disk_pos;
        test_node.cf = cf;
        %[T, status] = addToTranspositionTable(test_node, T);
        % Get the Zorbist Key for a state
        key = getZorbistKey(test_node, T.Zorbist);
        index = mod(key,T.size)+1;

        %check if a value already exists
        if (T.Table{index}(1).key == 0)
            T.Table{index}(1).key = key;
            T.Table{index}(1).cf = test_node.cf;
            %push to the queue
            PQ = push(PQ, vehicle_pos, disk_pos, disk_num, cf, [curr_node.vehicle_path,possible_push_point_paths{i}], curr_node.disk_path);
        else
            %check if current node is the same as stored node(s)
            j = 1;
            found = false;
            while ((j <= length(T.Table{index})) && (found == false))
                if (key == T.Table{index}(j).key)
                    found = true;
                    if ((T.Table{index}(j).visited == false) && (test_node.cf < T.Table{index}(j).cf))
                        %remove duplicate node
                        success = removeNodeFromQueue(PQ, vehicle_pos, disk_pos, T.Table{index}(j).cf);
                        %push to the queue
                        PQ = push(PQ, vehicle_pos, disk_pos, disk_num, cf, [curr_node.vehicle_path,possible_push_point_paths{i}], curr_node.disk_path);
                        T.Table{index}(j).cf = test_node.cf;
                    end
                end
                j = j + 1;
            end
            if (found == false)
                new_node.key = key;
                new_node.cf = test_node.cf;
                new_node.visited = false;
                T.Table{index}(end+1) = new_node;
                %push to the queue
                PQ = push(PQ, vehicle_pos, disk_pos, disk_num, cf, [curr_node.vehicle_path,possible_push_point_paths{i}], curr_node.disk_path);
            end
        end
    end
    
    
    VG = master_VG;
    PG = master_PG;
    CellNodeVG = master_CellNodeVG;
    PushPointsVGArray = master_PushPointsVGArray;
end
%need to update simulator to deal with multiple disks
if (sum(sort(curr_node.disk_pos) == sort(Map.goal_pos)) == length(Map.goal_pos))
    simulatePushResults(curr_node.vehicle_path, curr_node.disk_path, ...
      points_and_nodes, [length(nodes)+1, length(nodes)+length(push_points)], ...
       [length(nodes)+length(push_points)+1, length(points_and_nodes)], Map, 2, true);
    disp('solved')
else
    disp('unsolved')
end

%graph all different resulting graphs
% figure(3)
% subplot(2,2,1)
% title('Visibility Graph only nodes')
% plotMap(Map,3)
% plotVisibilityGraph(node_VG, nodes, 'b', 'm', 2);
% for i = 1:length(nodes)
%     plot(nodes(i,1),nodes(i,2), 'r*');
%     text(nodes(i,1),nodes(i,2),num2str(i));
% end
% subplot(2,2,2)
% title('Pushability Graph')
% plotMap(Map,3)
% plotPushabilityGraph(PG, nodes, 'b', 'm', 1);
% for i = 1:length(nodes)
%     plot(nodes(i,1),nodes(i,2), 'r*');
%     text(nodes(i,1),nodes(i,2),num2str(i));
% end
% subplot(2,2,3)
% title('Visibility Graph nodes and points')
% plotMap(Map,3)
% plotVisibilityGraph(VG, points_and_nodes, 'b', 'm', 1);
% for i = 1:length(points_and_nodes)
%     plot(points_and_nodes(i,1),points_and_nodes(i,2), 'r*');
%     text(points_and_nodes(i,1),points_and_nodes(i,2),num2str(i));
% end

end


%% priority queue functions

function PQ = createPQ(new_node, num_of_buckets, d_min)
    PQ.Queue(1) = dlnode(new_node);
    PQ.Queue(num_of_buckets) = dlnode();
    PQ.front_index = 1;
    PQ.num_of_buckets = num_of_buckets;
    PQ.lower_bound = 0;
    PQ.d_min = d_min;
return;
end

function [node, PQ] = pop(PQ)
    i = 1;
    front_index = PQ.front_index;
    while((i <= PQ.num_of_buckets) && (isempty(PQ.Queue(PQ.front_index).Data)))
        PQ.front_index = mod(front_index+i-1,PQ.num_of_buckets)+1;
        i = i + 1;
    end
        
    if(~isempty(PQ.Queue(PQ.front_index).Data))
        node = PQ.Queue(PQ.front_index).Data;
        if (~isempty(PQ.Queue(PQ.front_index).Next))
            tmp = PQ.Queue(PQ.front_index);
            PQ.Queue(PQ.front_index) = PQ.Queue(PQ.front_index).Next;
            removeNode(tmp);
        else
            PQ.Queue(PQ.front_index) = dlnode();
            PQ.lower_bound = PQ.lower_bound + PQ.d_min;
            PQ.front_index = mod(PQ.front_index , PQ.num_of_buckets) + 1;
        end
    else
        node = [];
        PQ = [];
    end
return;
end

function PQ = push(PQ, vehicle_pos, disk_pos, disk_num, cf, vehicle_path, disk_path)
    vehicle_path(end+1) = vehicle_pos;
    curr_disk_path = disk_path{disk_num};
    curr_disk_path(end+1) = disk_pos(disk_num);
    disk_path{disk_num} = curr_disk_path;
    node.vehicle_pos = vehicle_pos;
    node.disk_pos = disk_pos;
    node.vehicle_path = vehicle_path;
    node.disk_path = disk_path;
    node.cf = cf;
    index = mod(PQ.front_index + floor(cf-PQ.lower_bound/PQ.d_min)-1, PQ.num_of_buckets)+1;
    if (isempty(PQ.Queue(index).Data))
        PQ.Queue(index).Data = node;
    else
        new_node = dlnode(node);
        insertBefore(new_node, PQ.Queue(index));
        PQ.Queue(index) = new_node;
    end
return;
end

function empty = isQueueEmpty(PQ)
    i = 1;
    while((i <= PQ.num_of_buckets) && (isempty(PQ.Queue(i).Data)))
        i = i + 1;
    end
    if (i > PQ.num_of_buckets)
        empty = true;
    else
        empty = false;
    end
return;
end

function found = removeNodeFromQueue(PQ, veh_pos, disk_pos, cf)
    % find bucket that node would belong to
    index = mod(PQ.front_index + floor(cf-PQ.lower_bound/PQ.d_min)-1, PQ.num_of_buckets)+1;
    curr_node = PQ.Queue(index);
    found = false;
    while (~isempty(curr_node.Data))
        %find if current node is the node we're looking for
        data = curr_node.Data;
        if (veh_pos ==  data.vehicle_pos)
            j = 1;
            found_disk = true;
            while ((j <= length(disk_pos)) && (found_disk == true))
                if (isempty(find(data.disk_pos == disk_pos(j),1)))
                    found_disk = false;
                end
                j = j + 1;
            end
            if (found_disk == true)
                found = true;
                %remove the node from the Queue
                removeNode(curr_node)
            end
        end
        if (~isempty(curr_node.Next))
            curr_node = curr_node.Next;
        else
            curr_node = dlnode();
        end
    end
return;
end

function new_array = sortArray(array, sort_index)
    if (~isempty(array))
        cf_list = zeros(1,size(array,1));
        new_array = zeros(size(array));
        for i = 1:size(array,1)
            cf_list(i) = array(i,sort_index);
        end
        [~,ii] = sort(cf_list);
        for i = 1:size(array,1)
            new_array(i,:) = array(ii(i),:);
        end
    else
        new_array = [];
    end
return;
end

function index = pointInQueue(veh_pos, disk_pos, PQ)
    index  = 0;
    i = 1;
    while ((i <= length(PQ)) && (index == 0))
        curr_node = PQ(i);
        curr_veh_pos = curr_node.vehicle_pos;
        curr_disk_pos = curr_node.disk_pos;
        if (veh_pos ==  curr_veh_pos)
            j = 1;
            found_disk = true;
            while ((j <= length(disk_pos)) && (found_disk == true))
                if (isempty(find(curr_disk_pos == disk_pos(j),1)))
                    found_disk = false;
                end
                j = j + 1;
            end
            if (found_disk == true)
                index = i;
            end
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

function index = IsPointInArray(array, point)
    index = 0;
    i = 1;
    while ((i <= size(array,1)) && (index == 0))
        if ((array(i,1) == point(1)) && (array(i,2) == point(2)))
            index = i;
        end
        i = i + 1;
    end
return;
end