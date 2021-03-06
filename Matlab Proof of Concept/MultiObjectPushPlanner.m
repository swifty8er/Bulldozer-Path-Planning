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
for test_map_number = [1:34,37:52,55:59,63,64,66:69,71:73,76,79,81,82,84,85,92,94,96,100,53,54,60,61,62,65,70,74,75,77,80,83,86,87,89,90,91,95,98,35,88,93,97,99,36,78]
clearvars -except test_map_number Maps
close all
disp(test_map_number)
tic;
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
all_push_points = sortArray(all_push_points, 6);
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
CellPG = VGtoCellArray(PG);
% CellPushPointVG = VGtoCellArray((PushPointsVGArray(:,1:size(push_points,1)))');
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
curr_node.travelled = 0;
%calculate cost function
min_cf(1:length(curr_node.disk_pos)) = inf;
for m = 1:length(curr_node.disk_pos)
    for n = 1:length(Map.goal_pos)
        disk_pos = points_and_nodes(curr_node.disk_pos(m),:);
        goal_pos = points_and_nodes(Map.goal_pos(n),:);
        curr_cf = sqrt((disk_pos(1)-goal_pos(1))^2+(disk_pos(2)-goal_pos(2))^2)+curr_node.travelled;
        if (curr_cf < min_cf(m))
            min_cf(m) = curr_cf;
        end
    end
end
curr_node.cf = sum(min_cf);
goal_pos = points_and_nodes(Map.goal_pos,:);
%push into priority queue
[T, status] = addToTranspositionTable(curr_node, T);
PQ = curr_node;

%set up the master Visibility Graph and master Pushability Graph
master_VG = VG;
master_PG = PG;
master_CellNodeVG = CellNodeVG;
master_PushPointsVGArray = PushPointsVGArray;
%master_CellPG = CellPG;

%% A Star Section
%if the queue is not empty or the destination has not been reached then
%continue search
%(curr_node.disk_pos(1) == 20) && (curr_node.disk_pos(2) == 13) &&  (curr_node.disk_pos(3) == 16) && (curr_node.disk_pos(4) == 10) && (length(curr_node.disk_path{1,2}) > 1) && (curr_node.disk_path{1,2}(2) == 13)
while ((~isempty(PQ)) && (sum(sort(curr_node.disk_pos) ~= sort(Map.goal_pos))) && (toc <= 3600))
    [curr_node, PQ] = pop(PQ);
%     figure(11)
%     plotCurrentState(Map, curr_node, nodes, 1, 11)
%     clf(11);
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
    already_searched = false;
    while ((j <= length(T.Table{index})) && (found == false))
        if (key == T.Table{index}(j).key)
            found = true;
            if (T.Table{index}(j).visited == false)
                T.Table{index}(j).visited = true;
            else
                 already_searched = true;
            end
        end
        j = j + 1;
    end
    if (found == false)
        disp('error: curr node not found in transposition table')
    end
    if (already_searched == false)
        %Find the correct VG, PG to use by blocking paths that are inaccesible
        %due to disk placement
        for p = 1:length(curr_node.disk_pos)
            r = curr_node.disk_pos(p);
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
                    if (jj > length(nodes))
                        PushPointsVGArray(ii,jj - length(nodes)) = 0;
                    else
    %                         kk = find(CellPG{jj} == ii,1);
    %                         CellPG{jj}(kk) = [];
    %                         kk = find(CellPG{ii} == jj,1);
    %                         CellPG{ii}(kk) = [];
                            kk = find(CellNodeVG{jj} == ii,1);
                            CellNodeVG{jj}(kk) = [];
                            kk = find(CellNodeVG{ii} == jj,1);
                            CellNodeVG{ii}(kk) = [];
                    end
                end
                if (jj <= length(nodes))
                    if (ii > length(nodes))
                        PushPointsVGArray(jj,ii - length(nodes)) = 0;
                    end
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
                kk = find(CellNodeVG{s} == r,1);
                CellNodeVG{s}(kk) = [];
            end
    %         CellPG{r} = [];
    %         for s = 1:length(CellPG)
    %             kk = find(CellPG{s} == r,1);
    %             CellPG{s}(kk) = [];
    %         end
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
        %Find all possible valid push moves and add them to the queue
        [possible_push_points, possible_push_point_paths] = ...
            findReachablePushPoints(curr_vehicle_poses, curr_node.disk_pos, CellNodeVG,...
            PushPointsVGArray, all_push_points, PG, CellPG, NodesBlockVertices, Map.goal_pos, nodes, 2*Map.disc_radius);
        for i = 1:length(possible_push_points)
            %check if already been travelled to in the path and check if path is better
            vehicle_dest_pt = possible_push_points(i) + size(push_points,1);
            vehicle_pos = push_points(possible_push_points(i) - size(nodes,1),3);
            disk_num = find(curr_node.disk_pos == push_points(possible_push_points(i) - size(nodes,1),3));
            disk_pos = curr_node.disk_pos;
            disk_pos(disk_num) = dest_points(possible_push_points(i) - size(nodes,1),3);
            dest_pos = points_and_nodes(disk_pos,:);
            start_pos = points_and_nodes(curr_node.disk_pos,:);
            travelled = sqrt((dest_pos(disk_num,1)-start_pos(disk_num,1))^2+(dest_pos(disk_num,2)-start_pos(disk_num,2))^2);%+curr_node.travelled;
            ind_dist2goal = zeros(size(curr_node.disk_pos,2),1);
            for gg = 1:size(curr_node.disk_pos,2)
                ind_dist2goal(gg) = min(sqrt((dest_pos(gg,1)-goal_pos(:,1)).^2+(dest_pos(gg,2)-goal_pos(:,2)).^2));
            end

            %Check Transposition Table
            dist2goal = sum(ind_dist2goal);
            test_node.vehicle_pos = push_points(possible_push_points(i) - size(nodes,1),3);
            test_node.disk_pos = disk_pos;
            test_node.cf = travelled + dist2goal;
            %[T, status] = addToTranspositionTable(test_node, T);
            % Get the Zorbist Key for a state
            key = getZorbistKey(test_node, T.Zorbist);
            index = mod(key,T.size)+1;

            %check if a value already exists
            if (T.Table{index}(1).key == 0)
                T.Table{index}(1).key = key;
                T.Table{index}(1).cf = test_node.cf;
                %push to the queue
                PQ = push(PQ, vehicle_pos, vehicle_dest_pt, disk_pos, disk_num, travelled, [curr_node.vehicle_path,possible_push_point_paths{i}], curr_node.disk_path, dist2goal);
            else
                %check if current node is the same as stored node(s)
                j = 1;
                found = false;
                while ((j <= length(T.Table{index})) && (found == false))
                    if (key == T.Table{index}(j).key)
                        found = true;
                        if ((T.Table{index}(j).visited == false) && (test_node.cf < T.Table{index}(j).cf))
                            T.Table{index}(j).cf = test_node.cf;
                            %remove duplicate node
                            queue_index = pointInQueue(vehicle_pos, disk_pos, PQ);
                            if (queue_index ~= 0)
                                PQ(queue_index) = [];
                            else
                                disp("error: should not be possible");
                            end
                            %push to the queue
                            PQ = push(PQ, vehicle_pos, vehicle_dest_pt, disk_pos, disk_num, travelled, [curr_node.vehicle_path,possible_push_point_paths{i}], curr_node.disk_path, dist2goal);
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
                    PQ = push(PQ, vehicle_pos, vehicle_dest_pt, disk_pos, disk_num, travelled, [curr_node.vehicle_path,possible_push_point_paths{i}], curr_node.disk_path, dist2goal);
                end
            end
        end

        VG = master_VG;
        PG = master_PG;
        CellNodeVG = master_CellNodeVG;
        PushPointsVGArray = master_PushPointsVGArray;
        %CellPG = master_CellPG;
        PQ = sortQueue(PQ);
    end
end
%need to update simulator to deal with multiple disks
if (sum(sort(curr_node.disk_pos) == sort(Map.goal_pos)) == length(Map.goal_pos))
    simulatePushResults(curr_node.vehicle_path, curr_node.disk_path, ...
      points_and_nodes, [length(nodes)+1, length(nodes)+length(push_points)], ...
       [length(nodes)+length(push_points)+1, length(points_and_nodes)], Map, 2, true);
    disp('solved')
    disp(toc)
else
    disp('unsolved')
    disp(toc)
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

function PQ = push(PQ, vehicle_pos, vehicle_dest_pt, disk_pos, disk_num, travelled, vehicle_path, disk_path, dist2goal)
    vehicle_path(end+1) = vehicle_dest_pt;
    %vehicle_path(end+1) = vehicle_pos;
    curr_disk_path = disk_path{disk_num};
    curr_disk_path(end+1) = disk_pos(disk_num);
    disk_path{disk_num} = curr_disk_path;
    node.vehicle_pos = vehicle_pos;
    node.disk_pos = disk_pos;
    node.cf = travelled + dist2goal;
    node.vehicle_path = vehicle_path;
    node.disk_path = disk_path;
    node.travelled = travelled;
    PQ(length(PQ) + 1) = node;
return;
end

function new_PQ = sortQueue(PQ)
    if (~isempty(PQ))
        cf_list = zeros(1,length(PQ));
        new_PQ(length(PQ)) =  PQ(1);
        for i = 1:length(PQ)
            cf_list(i) = PQ(i).cf; %2nd most costly line
        end
        [~,ii] = sort(cf_list);
        for i = 1:length(PQ)
            new_PQ(i) = PQ(ii(i)); %Most costly line
        end
    else
        new_PQ = [];
    end
return;
end

function new_array = sortArray(array, sort_index)
    if (~isempty(array))
        cf_list = zeros(1,size(array,1));
        new_array = zeros(size(array));
        for i = 1:size(array,1)
            cf_list(i) = array(i,sort_index); %2nd most costly line
        end
        [~,ii] = sort(cf_list);
        for i = 1:size(array,1)
            new_array(i,:) = array(ii(i),:); %Most costly line
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
        %if (veh_pos ==  curr_veh_pos)
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
        %end
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