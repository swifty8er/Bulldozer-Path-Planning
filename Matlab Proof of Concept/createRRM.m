%function RRM = createRRM(CS)
    load("TestConfigSpace.mat");
    com_CS = imcomplement(CS);
    %show configuration space
    figure(1);
    imshow(CS);
    %find the medial axis of the configuration space
    MAT = bwmorph(com_CS,'skel',Inf);
    figure(2);
    imshow(MAT);
    %find distance transform of config space
    DT = bwdist(CS);
    figure(3);
    imshow(DT, []);
    colorbar
    colormap(jet(256));

    %find a list of points that line on MAT and their distances
    MA_List = [];
    DT_List = [];
    for i = 1:size(CS,1)
        for j = 1:size(CS,2)
            if(MAT(i,j) == 1)
                mat_pt.dist = DT(i,j);
                mat_pt.i = i;
                mat_pt.j = j;
                MA_List = [MA_List, mat_pt];
            elseif(DT(i,j)>0)
                dt_pt.dist = DT(i,j);
                dt_pt.i = i;
                dt_pt.j = j;
                DT_List = [DT_List, dt_pt];
            end
        end
    end
    %sort the list to find the most important MAT points
    MA_List = sortList(MA_List);
    DT_List = sortList(DT_List);
    MAandDTList = [MA_List, DT_List];

    %Usually he algorithm checks all cells can be seen after every new guard
    %   placement this will take too long for now.
    %We will only check if the guards can see all cells after all possible
    %   placements have been made.
    %If guards don't cover whole map then start using points from a sorted
    %   distance transform list

    %add first guard
    VisMap = zeros(size(CS,1),size(CS,2));
    Guard_Overlap = zeros(size(CS,1),size(CS,2)); %must make sure the num of guards is less than the number of bits in a double
    Guard_List = [MAandDTList(1).i,MAandDTList(1).j];
    VisMap = add2VisMap(CS, VisMap, Guard_List(1,:));
    guard_num = 1;
    Guard_Overlap = Guard_Overlap + isVisible(CS, Guard_List(1,:), 1); %guard num is between 1 and num of guards
    all_pts_visible = sum(sum(com_CS));
    i = 2;
    while ((i <= length(MAandDTList)) && (sum(sum(VisMap)) < all_pts_visible))
        %find new guard
        curr_guard = [MAandDTList(i).i,MAandDTList(i).j];
        seen = false;
        j = 1;
        while ((j <= size(Guard_List,1)) && (seen == false)) %guard can't see new guard 
            %Using the Bresenham’s Line Drawing Algorithm check each cell along
            %the line and see if there are any opaque cells
            seen = canSee(CS, curr_guard, Guard_List(j,:));
            %check next guard if opaque cell is found (guard can't see new
            %guard)
            j = j + 1;
        end
        if (seen == false)
            %keep new guard if not seen by other guards
            guard_num = guard_num + 1;
            Guard_List = [Guard_List; curr_guard];
            VisMap = add2VisMap(CS, VisMap, curr_guard);
            Guard_Overlap = Guard_Overlap + isVisible(CS, curr_guard, guard_num);
        end
        i = i + 1;
    end

    figure(4);
    Guard_Map = CS;
    for i = 1:size(Guard_List,1)
        Guard_Map(Guard_List(i,1),Guard_List(i,2)) = 2;
    end
    imshow(Guard_Map, []);
    colorbar
    colormap(jet(256));
    figure(5);
    imshow(VisMap, []);
    colorbar
    colormap(jet(256));

    GO_List = []; %list of all guard_overlaps
    for i = 1:size(Guard_Overlap,1)
        for j = 1:size(Guard_Overlap,2)
            %check if not an obstacle or unseen
            if (Guard_Overlap(i,j) > 0)
                %check if more than one guard sees this point
                curr_overlap = Guard_Overlap(i,j);
                if (1 && bitand(curr_overlap, curr_overlap-1))
                    go_pt.overlap = curr_overlap;
                    go_pt.i = i;
                    go_pt.j = j;
                    GO_List = [GO_List, go_pt];
                end
            end
        end
    end

    %find the numbers for all two guard combos
    all_guard_combos = zeros(guard_num*(guard_num-1)/2,3);
    for i = 1:length(all_guard_combos)
        a = floor((sqrt(8*i - 1) + 1)/2);
        b = i - 1 - a*(a - 1)/2;
        all_guard_combos(i,1) = 2^a + 2^b;
    end

    %find number of possible unique pairs
    for i = 1:length(GO_List)
        all_guard_combos(findBiPairs(GO_List(i).overlap, guard_num),3) = 1;
    end

    num_poss_connectors = sum(all_guard_combos(:,3));
    %all_guard_combos(:,3) = [];

    %find the best connector points
    connectors = [];
    connector_num = 0;
    connections_made = 0;
    i = 1;
    Roadmap = zeros(guard_num);
    while((i <= length(MAandDTList)) && (connections_made < num_poss_connectors))
        %check if not an obstacle or unseen
        j = 1;
        found = false;
        %find if in the Guard Overlap List
        while ((j <= length(GO_List)) && (found == false))
            if ((GO_List(j).i == MAandDTList(i).i) && (GO_List(j).j == MAandDTList(i).j))
                found = true;
                %check which guards can see this and determine if a connector
                %can be placed if one hasn't already been placed
                for k = 1:size(all_guard_combos,1)
                    %check if a connector has already been placed for a cetain
                    %combo
                    if (all_guard_combos(k,2) == 0)
                        %check if guard combo can see that cell
                        if (bitand(all_guard_combos(k,1),GO_List(j).overlap) == all_guard_combos(k,1))
                            %add connector
                            if ((isempty(connectors)) || (GO_List(j).i ~= connectors(end,1)) || (GO_List(j).j ~= connectors(end,2)))
                                connectors = [connectors; GO_List(j).i,GO_List(j).j];
                                connector_num = connector_num + 1;
                            end
                            %set up for graph
                            all_guard_combos(k,2) = 1;
                            guard_pair = findGuardPairs(all_guard_combos(k,1), guard_num);
                            guard1_pos = Guard_List(guard_pair(1),:);
                            guard2_pos = Guard_List(guard_pair(1),:);
                            connector_pos = [GO_List(j).i,GO_List(j).j];
                            Roadmap(guard_pair(1), guard_num + connector_num) = sqrt((guard1_pos(1)-connector_pos(1))^2+(guard1_pos(2)-connector_pos(2))^2);
                            Roadmap(guard_num + connector_num, guard_pair(1)) = sqrt((guard1_pos(1)-connector_pos(1))^2+(guard1_pos(2)-connector_pos(2))^2);
                            Roadmap(guard_pair(2), guard_num + connector_num) = sqrt((guard2_pos(1)-connector_pos(1))^2+(guard2_pos(2)-connector_pos(2))^2);
                            Roadmap(guard_num + connector_num, guard_pair(2)) = sqrt((guard2_pos(1)-connector_pos(1))^2+(guard2_pos(2)-connector_pos(2))^2);
                            connections_made = connections_made + 1;
                        end
                    end
                end
            end
            j = j + 1;
        end
        i = i + 1;
    end

    %find graph that finds the minimum spanning tree for connecting all guards using Prim's Minimum Spanning Tree Algorithm
    guards_visited = 1;
    visited_nodes = zeros(1, guard_num + connector_num);
    visited_edges = Roadmap;
    visited_nodes(1) = 1;
    first_push = 1;
    Test_Map = CS;
    for i = 1:guard_num + connector_num
        if(visited_edges(1, i) > 0)
            if (first_push == 1)
                node.from = 1;
                node.to = i;
                node.cf = Roadmap(1, i);
                PQ = node;
                first_push = 0;
                visited_edges(1,i) = 0;
                visited_edges(i,1) = 0;
            else
                PQ = push(PQ, 1, i, Roadmap(1, i));
                visited_edges(1,i) = 0;
                visited_edges(i,1) = 0;
            end
        end
    end
    PQ = sortQueue(PQ);
    figure(8);
    hold on;
    axis([0,100,0,100]);
    %while PQ empty and all guard nodes not added
    while((~isempty(PQ)) && (guards_visited < guard_num))
        [curr_node, PQ] = pop(PQ);
        %check if visiting a guard node
        if (curr_node.to <= guard_num)
            guards_visited = guards_visited + 1;
            search_num = guard_num + connector_num;
            visited_nodes(curr_node.to) = 1;
            visited_nodes(curr_node.from) = 1;
            from = connectors(curr_node.from-guard_num,:);
            to = Guard_List(curr_node.to,:);
            plot([from(1),to(1)],[from(2),to(2)], 'r');
        else
            search_num = guard_num;
            from = Guard_List(curr_node.from,:);
            to = connectors(curr_node.to-guard_num,:);
            plot([from(1),to(1)],[from(2),to(2)], 'b');
        end
        %add all relevant edges to the queue for the node pointed to by this edge
        for i = 1:search_num
            if(visited_edges(curr_node.to, i) > 0)
                PQ = push(PQ, curr_node.to, i, Roadmap(curr_node.to, i));
                visited_edges(curr_node.to,i) = 0;
                visited_edges(i,curr_node.to) = 0;
            end
        end
        PQ = sortQueue(PQ);
    end

    figure(6);
    Node_Map = Guard_Map;
    for i = 1:size(connectors,1)
        Node_Map(connectors(i,1),connectors(i,2)) = 2;
    end
    imshow(Node_Map, []);
    colorbar
    colormap(jet(256));

    figure(7);
    Node_Map = Guard_Map;
    for i = 1:size(connectors,1)
        if (visited_nodes(guard_num+i) == 1)
            Node_Map(connectors(i,1),connectors(i,2)) = 2;
        end
    end
    imshow(Node_Map, []);
    colorbar
    colormap(jet(256));

    figure(8)
    con_plot = Guard_List;
    for i = 1:size(connectors,1)
        if (visited_nodes(guard_num+i) == 1)
            con_plot = [con_plot;connectors(i,1),connectors(i,2)];
        end
    end
    plot(con_plot(:,1),con_plot(:,2),'g*')

    %remove any connectors that are not used
    i =1;
    j = 0;
    while(i <= size(connectors,1))
        if (visited_nodes(guard_num+i+j) == 0)
            Roadmap(guard_num+i,:) = [];
            Roadmap(:,guard_num+i) = [];
            connectors(i,:) = [];
            j = j + 1;
        else
            i = i + 1;
        end
    end

    %create a new graph that connects all possible paths between connectors and
    %guards
    all_nodes = [Guard_List;connectors];
    first_edge = true;
    all_edges = [];
    for i = 1:length(all_nodes)-1
        for j = i+1:length(all_nodes)
            seen = canSee(CS, all_nodes(i,:), all_nodes(j,:));
            if(seen == true)
                conn1 = all_nodes(i,:);
                conn2 = all_nodes(j,:);
                dist = sqrt((conn1(1)-conn2(1))^2+(conn1(2)-conn2(2))^2);
                Roadmap(i, j) = dist;
                Roadmap(j, i) = dist;
                edge.pt1 = i;
                edge.pt2 = j;
                edge.dist = dist;
                if (first_edge == true)
                    all_edges = edge;
                    first_edge = false;
                else
                    all_edges(end+1) = edge;
                end
            end
        end
    end

    %sort the edge list for the Kruskal algorithm
    dist_list = zeros(1,length(all_edges));
    tmp(length(all_edges)) = all_edges(1);
    for i = 1:length(all_edges)
        dist_list(i) = all_edges(i).dist;
    end
    [~,ii] = sort(dist_list);
    for i = 1:length(all_edges)
        tmp(i) = all_edges(ii(i));
    end
    all_edges = tmp;


    figure(9);
    hold on;
    axis([0,size(CS,2),0,size(CS,1)]);
    plot(all_nodes(:,1),all_nodes(:,2),'g*')
    for i = 1:size(all_nodes,1)
        text(all_nodes(i,1),all_nodes(i,2),num2str(i));
    end
    for i = 1:size(Roadmap,1)-1
        for j = i+1:size(Roadmap,2)
            if (Roadmap(i,j) > 0)
                x = [all_nodes(i,1),all_nodes(j,1)];
                y = [all_nodes(i,2),all_nodes(j,2)];
                plot(x,y,'b');
            end
        end
    end
    %find a new minimum spanning tree using Kruskal's algorithm
    num_edges = 0;
    nodes_visited.visited = zeros(size(all_nodes,1),1);
    nodes_visited.set_num = zeros(size(all_nodes,1),1);
    Final_RM = zeros(size(Roadmap));
    i = 1;
    set_num = 1;
    while ((~isempty(all_edges)) && (num_edges < size(all_nodes,1)-1))
        curr_edge = all_edges(i);
        %check if both nodes haven't been visited
        if ((nodes_visited.visited(curr_edge.pt1) == 0) && (nodes_visited.visited(curr_edge.pt2) == 0))
            nodes_visited.visited(curr_edge.pt1) = 1;
            nodes_visited.visited(curr_edge.pt2) = 1;
            nodes_visited.set_num(curr_edge.pt1) = set_num;
            nodes_visited.set_num(curr_edge.pt2) = set_num;
            set_num = set_num + 1;
            num_edges = num_edges + 1;
            Final_RM(curr_edge.pt1,curr_edge.pt2) = 1;
            Final_RM(curr_edge.pt2,curr_edge.pt1) = 1;
            %check if both nodes have been visited but are part of different
            %trees
        elseif ((nodes_visited.visited(curr_edge.pt1) == 1) && (nodes_visited.visited(curr_edge.pt2) == 1))
            if (nodes_visited.set_num(curr_edge.pt1) ~= nodes_visited.set_num(curr_edge.pt2))
                num_edges = num_edges + 1;
                Final_RM(curr_edge.pt1,curr_edge.pt2) = 1;
                Final_RM(curr_edge.pt2,curr_edge.pt1) = 1;
                prev_set = nodes_visited.set_num(curr_edge.pt2);
                for k = 1:size(nodes_visited.set_num,1)
                    if (nodes_visited.set_num(k) == prev_set)
                        nodes_visited.set_num(k) = nodes_visited.set_num(curr_edge.pt1);
                    end
                end
            end
            %checks if one node is already part of a set a joins that set
        elseif (nodes_visited.visited(curr_edge.pt1) == 0)
            nodes_visited.visited(curr_edge.pt1) = 1;
            nodes_visited.set_num(curr_edge.pt1) = nodes_visited.set_num(curr_edge.pt2);
            num_edges = num_edges + 1;
            Final_RM(curr_edge.pt1,curr_edge.pt2) = 1;
            Final_RM(curr_edge.pt2,curr_edge.pt1) = 1;
        elseif (nodes_visited.visited(curr_edge.pt2) == 0)
            nodes_visited.visited(curr_edge.pt2) = 1;
            nodes_visited.set_num(curr_edge.pt2) = nodes_visited.set_num(curr_edge.pt1);
            num_edges = num_edges + 1;
            Final_RM(curr_edge.pt1,curr_edge.pt2) = 1;
            Final_RM(curr_edge.pt2,curr_edge.pt1) = 1;
        else
            disp('WHAT?')
        end
        i = i +1;
    end

    figure(10);
    hold on;
    axis([0,size(CS,2),0,size(CS,1)]);
    plot(all_nodes(:,1),all_nodes(:,2),'g*')
    for i = 1:size(Final_RM,1)-1
        for j = i+1:size(Final_RM,2)
            if (Final_RM(i,j) > 0)
                x = [all_nodes(i,1),all_nodes(j,1)];
                y = [all_nodes(i,2),all_nodes(j,2)];
                plot(x,y,'b');
            end
        end
    end

    %add useful nodes
    original_nodes = all_nodes;
    for i = 1:length(MA_List)
        curr_node = [MA_List(i).i, MA_List(i).j];
        closest_nodes = zeros(2,1);
        closest_dist = inf;
        closest_dist2 = inf;
        already_node = false;
        j = 1;
        while((j <=size(original_nodes,1)) && (already_node == false))
            if ((curr_node(1) == original_nodes(j,1)) && (curr_node(2) == original_nodes(j,2)))
                already_node = true;
            end
            j = j + 1;
        end
        if (already_node == false)
            for j = 1:size(original_nodes,1)
                %find the two closest nodes to the MA point
                seen = canSee(CS, curr_node, original_nodes(j,:));
                curr_dist = sqrt((curr_node(1)-original_nodes(j,1))^2+(curr_node(2)-original_nodes(j,2))^2);
                if((seen == true) && (curr_dist < closest_dist2))
                    if(curr_dist < closest_dist)
                        closest_dist2 = closest_dist;
                        closest_dist = curr_dist;
                        closest_nodes(2) = closest_nodes(1);
                        closest_nodes(1) = j;
                    else
                        closest_dist2 = curr_dist;
                        closest_nodes(2) = j;
                    end
                end
            end
            %find the shortest path between the two recently found clostest points
            %doesn't include start and end points
            if ((closest_nodes(1) > 0) && (closest_nodes(2) > 0))
                curr_path = findPathInGraph(closest_nodes(1), closest_nodes(2), all_nodes, Final_RM);
                curr_path(1,:) = [];
                curr_path(end,:) = [];
                %find if all nodes in the shortest path can't see the new potential
                %node
                j = 1;
                if (~isempty(curr_path))
                    seen = true;
                    while((j <=size(curr_path,1)) && (seen == true))
                        curr_path_node = curr_path(j,:);
                        seen = canSee(CS, curr_node, curr_path_node);
                        j = j + 1;
                    end
                    %if can't see new node then add node to graph
                    if(seen == false)
                        all_nodes = [all_nodes; curr_node];
                        Final_RM(end+1, end+1) = 0;
                        Final_RM(end, closest_nodes(1)) = 1;
                        Final_RM(closest_nodes(1), end) = 1;
                        Final_RM(end, closest_nodes(2)) = 1;
                        Final_RM(closest_nodes(2), end) = 1;
                    end
                end
            end
        end
    end

    figure(11);
    hold on;
    axis([0,size(CS,2),0,size(CS,1)]);
    plot(all_nodes(:,1),all_nodes(:,2),'g*')
    for i = 1:size(all_nodes,1)
        text(all_nodes(i,1),all_nodes(i,2),num2str(i));
    end

    %remove nodes with not enough reconnected edges
    %add all edges to a priority queue
    first = true;
    PQ = [];
    edge = [];
    for i = 1:size(all_nodes,1)-1
        for j = i+1:size(all_nodes,1)
            from = all_nodes(i,:);
            to = all_nodes(j,:);
            seen = canSee(CS, from, to);
            if(seen == true)
                if (first == true)
                    edge.from = i;
                    edge.to = j;
                    edge.cf = sqrt((from(1)-to(1))^2+(from(2)-to(2))^2);
                    PQ = edge;
                    first = false;
                else
                    dist = sqrt((from(1)-to(1))^2+(from(2)-to(2))^2);
                    PQ = push(PQ, i, j, dist);
                end
            end
        end
    end

    %find the edges that are useful
    K = 1.5; %K-useful constant
    New_Final_RM = zeros(size(Final_RM));
    New_Final_RM(1:size(Roadmap,1),1:size(Roadmap,2)) = Final_RM(1:size(Roadmap,1),1:size(Roadmap,2));
    while(~isempty(PQ))
        [edge, PQ] = pop(PQ);
        curr_path = findPathInGraph(edge.from, edge.to, all_nodes, Final_RM);
        %find length of path
        graph_dist = 0;
        for i = 1:size(curr_path,1)-1
            from = curr_path(i,:);
            to = curr_path(i+1,:);
            graph_dist = graph_dist + sqrt((from(1)-to(1))^2+(from(2)-to(2))^2);
        end
        if(K*edge.cf < graph_dist)
            from = all_nodes(edge.from,:);
            to = all_nodes(edge.to,:);
            seen = canSee(CS, from, to);
            if(seen == true)
                New_Final_RM(edge.from,edge.to) = 1;
                New_Final_RM(edge.to,edge.from) = 1;
            end
        end
    end

    %clean up any new nodes that don't have more than two edges
    k = 0;
    i = size(Roadmap,1)+1;
    while (i <= size(New_Final_RM,1))
        num_edges = 0;
        j = 1;
        while ((j <= size(Roadmap,2)) && (num_edges < 2))
            if(New_Final_RM(i,j) > 0)
                num_edges = num_edges + 1;
            end
            j = j +1 ;
        end
        if (num_edges < 2)
            New_Final_RM(i,:) = [];
            New_Final_RM(:,i) = [];
            all_nodes(i,:) = [];
            k = k + 1;
            i = size(Roadmap,1);
        end
        i = i + 1;
    end

    figure(12);
    hold on;
    axis([0,size(CS,2),0,size(CS,1)]);
    plot(all_nodes(:,1),all_nodes(:,2),'g*')
    for i = 1:size(New_Final_RM,1)-1
        for j = i+1:size(New_Final_RM,2)
            if (New_Final_RM(i,j) > 0)
                x = [all_nodes(i,1),all_nodes(j,1)];
                y = [all_nodes(i,2),all_nodes(j,2)];
                plot(x,y,'b');
            end
        end
    end

    RRM = all_nodes;
    
    figure(13);
    RRM_Map = CS;
    for i = 1:size(all_nodes,1)
        RRM_Map(all_nodes(i,1),all_nodes(i,2)) = 2;
    end
    imshow(RRM_Map, []);
    colorbar
    colormap(jet(256));

%return;
%end


function new_List = sortList(List)
    if (~isempty(List))
        dist_list = zeros(1,length(List));
        new_List(length(List)) = List(1);
        for i = 1:length(List)
            dist_list(i) = List(i).dist;
        end
        [~,ii] = sort(dist_list,'descend');
        for i = 1:length(List)
            new_List(i) = List(ii(i));
        end
    else
        new_List = [];
    end
return;
end

%show all points on the CS map that are visible from the guard
function reachable = isVisible(CS, guard, guard_num)
    reachable = zeros(size(CS,1),size(CS,2));
    for i = 1:size(CS,1)
        for j = 1:size(CS,2)
            if (CS(i,j) == 0)
                seen = canSee(CS, guard, [i,j]);
                if (seen == true)
                    reachable(i,j) = 2^(guard_num-1);
                end
            end
        end
    end
return;
end

%investigates unseen points and fills them if they have been seen
%updating the Vis Map
%A speed can be completed by filling out the Vis Map everytime the canSee
%function is called not only filling the in the point being tested but also
%the points inbetween the guard and the point being tested
function VisMap = add2VisMap(CS, VisMap, guard)
    for i = 1:size(CS,1)
        for j = 1:size(CS,2)
            if (CS(i,j) == 0)
                if (VisMap(i,j) == 0)
                    seen = canSee(CS, guard, [i,j]);
                    if (seen == true)
                        VisMap(i,j) = 1;
                    end
                end
            end
        end
    end
return;
end

function guard_pairs = findGuardPairs(num, num_of_bits)
    guard_pairs = [];
    for i = 1:num_of_bits-1
        for j = i+1:num_of_bits
            %check if both bits being checked are on
            if (bitget(num,i) + bitget(num,j) == 2)
                %bi_pairs = [bi_pairs; 2^(i-1) + 2^(j-1)];
                guard_pairs = [guard_pairs; i, j]; %returns the indexes
            end
        end
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

function PQ = push(PQ, from, to, cf)
    node.from = from;
    node.to = to;
    node.cf = cf;
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