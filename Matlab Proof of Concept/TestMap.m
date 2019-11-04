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

%set constants
close all;
clear all;
Map.min_x = 0;
Map.max_x = 7;
Map.min_y = 0;
Map.max_y = 7;
Map.grid_size = 0.025;
Map.boundary = [0,0;0,7;7,7;7,0;0,0];
Map.disc_radius = 0.45;
Map.vehicle_radius = 0.45;
%create test map
Map = createTestMap(Map);
%plot test map
plotMap(Map,1);
%Create configuration space
CS = Map2CS(Map);
%create RRM
%RRM = createRRM(CS);
RRM = medialAxisPRM(CS, Map);
%hard coded graph nodes
%nodes = [0.5,5.5;0.5,6.5;4.5,6.5;4.5,5.5;4.5,4.5;5.5,4.5;6.5,4.5;...
%          6.5,3.5;6.5,1.5;5.5,1.5;4.5,1.5;4.5,0.5;3.5,1.5;2.5,1.5;...
%          1.5,1.5;0.5,1.5;0.5,0.5];
%change to useable x-y coordinates
nodes(:,1) = (RRM(:,2)-0.5)*Map.grid_size;
nodes(:,2) = Map.max_y - (RRM(:,1)-0.5)*Map.grid_size;
plotMap(Map,1);
for i = 1:length(nodes)
    plot(nodes(i,1),nodes(i,2), 'r*');
    text(nodes(i,1),nodes(i,2),num2str(i));
end

[VG, Cond_Vertices] = createVisibilityGraph(nodes, Map);
plotVisibilityGraph(VG, nodes, 'b', 'c', 2);

%find a list of pushable points and a pushabilitiy graph
node_VG = VG;
[PG, all_push_points] = createPushabilityGraph(VG, nodes, Map);

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

%% randomly choose vehicle, disk and goal pos
% pushable_nodes = [];
% destination_nodes = [];
% for i = 1:length(all_push_points)
%     if (isempty(find(pushable_nodes == all_push_points(i, 3),1)))
%         pushable_nodes(end+1) = all_push_points(i, 3);
%     end
%     if (isempty(find(destination_nodes == all_push_points(i, 6),1)))
%         destination_nodes(end+1) = all_push_points(i, 6);
%     end
% end
% destination_nodes = sort(destination_nodes);
% 
% Map.initial_vehicle_pos = round(rand*length(nodes)); %working: 8
% %while loops to make sure a node that is already filled isn't chosen
% Map.initial_disk_pos = pushable_nodes(round(rand*length(pushable_nodes))); %working: 6
% while (Map.initial_disk_pos == Map.initial_vehicle_pos)
%     Map.initial_disk_pos = pushable_nodes(round(rand*length(pushable_nodes)));
% end
% Map.goal_pos = destination_nodes(round(rand*length(destination_nodes))); %working: 3
% while ((Map.goal_pos == Map.initial_vehicle_pos) || (Map.goal_pos == Map.initial_disk_pos))
%     Map.goal_pos = destination_nodes(round(rand*length(destination_nodes)));
% end
% % Map.initial_vehicle_pos = 3;
% % Map.initial_disk_pos = 5;
% % Map.goal_pos = 17;
% 
% %return the possible push points the vehicle can reach from its start
% %position
% %[possible_push_points, possible_push_point_paths] = findReachablePushPoints(Map.initial_vehicle_pos, Map.initial_disk_pos, CellNodeVG, PushPointsVGArray, push_points);
% 
% %A star search to find solution
% 
% %initial state
% curr_node.vehicle_pos = Map.initial_vehicle_pos;
% curr_node.disk_pos = Map.initial_disk_pos;
% curr_node.vehicle_path = Map.initial_vehicle_pos;
% curr_node.disk_path{1} = Map.initial_disk_pos;
% curr_node.travelled = 0;
% disk_pos = points_and_nodes(curr_node.disk_pos,:);
% goal_pos = points_and_nodes(Map.goal_pos,:);
% curr_node.cf = sqrt((disk_pos(1)-goal_pos(1))^2+(disk_pos(2)-goal_pos(2))^2)+curr_node.travelled;
% %push into priority queue
% PQ = curr_node;
% 
% %if the queue is not empty or the destination has not been reached then
% %continue search
% while ((~isempty(PQ)) && (curr_node.disk_pos ~= Map.goal_pos))
%     [curr_node, PQ] = pop(PQ);
%     curr_vehicle_poses = [];
%     %find possible push points reachable by the vehicle
%     if (curr_node.vehicle_pos > length(nodes))
%         for i = 1:length(nodes)
%             if (VG(curr_node.vehicle_pos,i) ~= 0)
%                 curr_vehicle_poses(end+1) = i;
%             end
%         end
%     else
%         curr_vehicle_poses = curr_node.vehicle_pos;
%     end
%     [possible_push_points, possible_push_point_paths] = ...
%         findReachablePushPoints(curr_vehicle_poses, curr_node.disk_pos, CellNodeVG, PushPointsVGArray, push_points);
%     for i = 1:length(possible_push_points)
%         %check if already been travelled to in the path
%         
%         %check if path is better
%         
%         %push to the queue
%         vehicle_pos = possible_push_points(i) + size(push_points,1);
%         disk_pos = dest_points(possible_push_points(i) - size(nodes,1),3);
%         dest_pos = points_and_nodes(disk_pos,:);
%         start_pos = points_and_nodes(curr_node.disk_pos,:);
%         travelled = sqrt((dest_pos(1)-start_pos(1))^2+(dest_pos(2)-start_pos(2))^2)+curr_node.travelled;
%         dist2goal = sqrt((dest_pos(1)-goal_pos(1))^2+(dest_pos(2)-goal_pos(2))^2);
%         PQ = push(PQ, vehicle_pos, disk_pos, 1, travelled, [curr_node.vehicle_path,possible_push_point_paths{i}], curr_node.disk_path, dist2goal);
%     end
%     PQ = sortQueue(PQ);
% end
% if (curr_node.disk_pos == Map.goal_pos)
%     simulatePushResults(curr_node.vehicle_path, curr_node.disk_path, ...
%       points_and_nodes, [length(nodes)+1, length(nodes)+length(push_points)], ...
%        [length(nodes)+length(push_points)+1, length(points_and_nodes)], Map, 2);
% end
% 
% %graph all different resulting graphs
% figure(3)
% subplot(2,2,1)
% title('Visibility Graph only nodes')
% plotMap(Map,3)
% plotVisibilityGraph(node_VG, nodes, 'b' , 2);
% for i = 1:length(nodes)
%     plot(nodes(i,1),nodes(i,2), 'r*');
%     text(nodes(i,1),nodes(i,2),num2str(i));
% end
% subplot(2,2,2)
% title('Pushability Graph')
% plotMap(Map,3)
% plotPushabilityGraph(PG, nodes, 'b' , 1);
% for i = 1:length(nodes)
%     plot(nodes(i,1),nodes(i,2), 'r*');
%     text(nodes(i,1),nodes(i,2),num2str(i));
% end
% subplot(2,2,3)
% title('Visibility Graph nodes and points')
% plotMap(Map,3)
% plotVisibilityGraph(VG, points_and_nodes, 'b' , 1);
% for i = 1:length(points_and_nodes)
%     plot(points_and_nodes(i,1),points_and_nodes(i,2), 'r*');
%     text(points_and_nodes(i,1),points_and_nodes(i,2),num2str(i));
% end

%% sim for multiple disks

% %pick a vehicle
% disp('choose a vehicle position');
% Map.initial_vehicle_pos = pickNode(nodes,1);
% num_disks = 2;
% %pick 2 (can be changed) disk nodes
% for i = 1:num_disks
%     disp('choose a disk position for disk number: ');
%     disp(i);
%     Map.initial_disk_pos(i) = pickNode(nodes,1); %[6;10;16]
%     used_disk_nodes = Map.initial_disk_pos;
%     used_disk_nodes(i) = [];
%     while ((Map.initial_vehicle_pos == Map.initial_disk_pos(i)) || ...
%             (~isempty(find(used_disk_nodes == Map.initial_disk_pos(i),1))))
%         disp('must chose a unique node');
%         Map.initial_disk_pos(i) = pickNode(nodes,1);
%     end
% end
% 
% %pick 2 goal nodes
% for i = 1:num_disks
%     disp('choose a goal position for goal number: ');
%     disp(i);
%     Map.goal_pos(i) = pickNode(nodes,1); %[3;1;16]
%     used_goal_nodes = Map.goal_pos;
%     used_goal_nodes(i) = [];
%     while (~isempty(find(used_goal_nodes == Map.goal_pos(i),1)))
%         disp('must chose a unique node');
%         Map.goal_pos(i) = pickNode(nodes,1);
%     end
% end

%two disk example
% Map.initial_vehicle_pos = 11;
% Map.initial_disk_pos = [20;23];
% Map.goal_pos = [13;3];

%three disk example
Map.initial_vehicle_pos = 12;
Map.initial_disk_pos = [14;27;33];
Map.goal_pos = [8;31;33];

%four disk example
% Map.initial_vehicle_pos = 8;
% Map.initial_disk_pos = [4;6;10;16];
% Map.goal_pos = [2;3;7;17];

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

%if the queue is not empty or the destination has not been reached then
%continue search
while ((~isempty(PQ)) && (sum(sort(curr_node.disk_pos) ~= sort(Map.goal_pos))))
    [curr_node, PQ] = pop(PQ);
    %Find the correct VG, PG to use by blocking paths that are inaccesible
    %due to disk placement
    for p = 1:length(curr_node.disk_pos)
        r = curr_node.disk_pos(p);
        for q = 1:size(NodesBlockVertices(r).vertices,1)
            ii = NodesBlockVertices(r).vertices(q,1);
            jj = NodesBlockVertices(r).vertices(q,2);
            VG(ii, jj) = 0;
            VG(jj, ii) = 0;
            PG(ii, jj) = 0;
            PG(jj, ii) = 0;
            if (ii <= length(nodes))
                PushPointsVGArray(ii,jj) = 0;
                kk = find(CellNodeVG{ii} == jj);
                CellNodeVG{ii}(kk) = [];
            end
            if (jj <= length(nodes))
                PushPointsVGArray(jj,ii) = 0;
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
        PushPointsVGArray(:,r) = 0;
        CellNodeVG{r} = [];
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
        findReachablePushPoints(curr_vehicle_poses, curr_node.disk_pos, CellNodeVG, PushPointsVGArray, all_push_points);
    for i = 1:length(possible_push_points)
        %check if already been travelled to in the path and check if path is better
        vehicle_pos = possible_push_points(i) + size(push_points,1);
        disk_num = find(curr_node.disk_pos == push_points(possible_push_points(i) - size(nodes,1),3));
        disk_pos = curr_node.disk_pos;
        disk_pos(disk_num) = dest_points(possible_push_points(i) - size(nodes,1),3);
        dest_pos = points_and_nodes(disk_pos,:);
        start_pos = points_and_nodes(curr_node.disk_pos,:);
        travelled = sqrt((dest_pos(1)-start_pos(1))^2+(dest_pos(2)-start_pos(2))^2)+curr_node.travelled;
        dist2goal = min(sqrt((dest_pos(1)-goal_pos(:,1)).^2+(dest_pos(2)-goal_pos(:,2)).^2));
        test_node.vehicle_pos = vehicle_pos;
        test_node.disk_pos = disk_pos;
        test_node.cf = travelled + dist2goal;
        [T, status] = addToTranspositionTable(test_node, T);
        if ((status == 'E') || (status == 'R'))
            %push to the queue
            PQ = push(PQ, vehicle_pos, disk_pos, disk_num, travelled, [curr_node.vehicle_path,possible_push_point_paths{i}], curr_node.disk_path, dist2goal);
            if (status == 'R')
                %remove duplicate node
                index = pointInQueue(vehicle_pos, disk_pos, PQ);
                if (index ~= 0)
                    PQ(index) = [];
                else
                    disp("error: should not be possible");
                end
            end
        end
    end
    VG = master_VG;
    PG = master_PG;
    CellNodeVG = master_CellNodeVG;
    PushPointsVGArray = master_PushPointsVGArray;
    PQ = sortQueue(PQ);
end
%need to update simulator to deal with multiple disks
if (sum(sort(curr_node.disk_pos) == sort(Map.goal_pos)) == length(Map.goal_pos))
    simulatePushResults(curr_node.vehicle_path, curr_node.disk_path, ...
      points_and_nodes, [length(nodes)+1, length(nodes)+length(push_points)], ...
       [length(nodes)+length(push_points)+1, length(points_and_nodes)], Map, 2);
end

%graph all different resulting graphs
figure(3)
subplot(2,2,1)
title('Visibility Graph only nodes')
plotMap(Map,3)
plotVisibilityGraph(node_VG, nodes, 'b', 'm', 2);
for i = 1:length(nodes)
    plot(nodes(i,1),nodes(i,2), 'r*');
    text(nodes(i,1),nodes(i,2),num2str(i));
end
subplot(2,2,2)
title('Pushability Graph')
plotMap(Map,3)
plotPushabilityGraph(PG, nodes, 'b', 'm', 1);
for i = 1:length(nodes)
    plot(nodes(i,1),nodes(i,2), 'r*');
    text(nodes(i,1),nodes(i,2),num2str(i));
end
% subplot(2,2,3)
% title('Visibility Graph nodes and points')
% plotMap(Map,3)
% plotVisibilityGraph(VG, points_and_nodes, 'b', 'm', 1);
% for i = 1:length(points_and_nodes)
%     plot(points_and_nodes(i,1),points_and_nodes(i,2), 'r*');
%     text(points_and_nodes(i,1),points_and_nodes(i,2),num2str(i));
% end




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

function PQ = push(PQ, vehicle_pos, disk_pos, disk_num, travelled, vehicle_path, disk_path, dist2goal)
    vehicle_path(end+1) = vehicle_pos;
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