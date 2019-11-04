%useful image scripts

%image 1: Connections of All Push Points with Nodes
%nodes([10,14,18,28],:) = [];
new_nodes = nodes;
new_nodes([10,14,18,28],:) = [];
plotMap(Map,1);

plotVisibilityGraph(VG, points_and_nodes, 'y', 'c' , 4);

for i = 1:length(new_nodes)
    plot(new_nodes(i,1),new_nodes(i,2), 'r*','MarkerSize',24);
    %text(nodes(i,1),nodes(i,2),num2str(i), 'FontSize', 24);
end

new_pp = [];
for i = 1:size(push_points,1)-1
    j = i+1;
    in_tolerance = true;
    while ((j <= size(push_points,1)) && (in_tolerance == true))
        dist = ptDist(push_points(i,:), push_points(j,:));
        if (dist < 0.2)
            in_tolerance = false;
        end
        j = j+1;
    end
    if (in_tolerance == true)
        new_pp = [new_pp; push_points(i,:)];
    end
end

for i = 1:size(new_pp,1)
    plot([new_pp(i,1), nodes(new_pp(i,3),1)], [new_pp(i,2), nodes(new_pp(i,3),2)], 'k:','LineWidth',2);
    %text(new_pp(i,1),new_pp(i,2),num2str(i+length(nodes)), 'FontSize', 24);
end

for i = 1:size(new_pp,1)
    plot(new_pp(i,1), new_pp(i,2), 'b*','MarkerSize',24);
    %text(new_pp(i,1),new_pp(i,2),num2str(i+length(nodes)), 'FontSize', 24);
end



%image 2: Demonstration of PG
plotMap(Map,1);
new_nodes = nodes;
goal_pos = new_nodes(Map.goal_pos,:);
for i = 1:size(goal_pos,1)
    goal_circle = circlePoints(goal_pos(i,:), Map.disc_radius*1.1, 25);
    plot(goal_circle(:,1),goal_circle(:,2),'g', 'LineWidth',3);
end

new_lines = all_push_points([1,3,5,6,10,13,16,21,29,30,36,42,43,44,55,56,57,63,85], [3,6]);
for i = 1:size(new_lines,1)
    quiver(new_nodes(new_lines(i,1),1),new_nodes(new_lines(i,1),2),new_nodes(new_lines(i,2),1)-new_nodes(new_lines(i,1),1) ...
        ,new_nodes(new_lines(i,2),2)-new_nodes(new_lines(i,1),2), 0, 'g--', 'LineWidth',5)
end

new_nodes([18,14,10,28],:) = [];
for i = 1:length(new_nodes)
    plot(new_nodes(i,1),new_nodes(i,2), 'r*','MarkerSize',24);
    %text(new_nodes(i,1),new_nodes(i,2),num2str(i), 'FontSize', 24);
end
 
new_pp = push_points([1,3,5,6,10,13,16,21,29,30,36,42,43,44,55,56,57,63,85], 1:3);

for i = 1:size(new_pp,1)
    plot([new_pp(i,1), nodes(new_pp(i,3),1)], [new_pp(i,2), nodes(new_pp(i,3),2)], 'k:','LineWidth',2);
    %text(new_pp(i,1),new_pp(i,2),num2str(i+length(nodes)), 'FontSize', 24);
end

for i = 1:size(new_pp,1)
    plot(new_pp(i,1), new_pp(i,2), 'b*','MarkerSize',24);
    %text(new_pp(i,1),new_pp(i,2),num2str(i+length(new_nodes)), 'FontSize', 24);
end






%image 3: Finding Possible Push Points
figure(11); hold on;
plotCurrentState(Map, curr_node, nodes, 3, 11)
linestoplot = [5.5,5.5;5.4,5.5;5.5,5.5;5.5,3.5;6.5,3.4;5.5,3.5;5.5,1.5;6.5,1.6;5.5,1.5;3.5,1.5;3.5,5.5;3.6,5.5;2.5,5.5;2.5,6.5;0.5,6.5;0.5,5.5;0.6,5.5];
plot(linestoplot(:,1),linestoplot(:,2),'c')
plot(linestoplot(:,1),linestoplot(:,2),'c','LineWidth',5)
linestoplot = [5.5,5.5;5.5,6.5;7.5,6.5;7.4,4.5];
plot(linestoplot(:,1),linestoplot(:,2),'c','LineWidth',5)
curr_pos = nodes(curr_node.vehicle_pos,:);
vehicle_circle = circlePoints(curr_pos, Map.disc_radius, 25);
plot(vehicle_circle(:,1),vehicle_circle(:,2),'r', 'LineWidth',3)
new_pp = push_points(possible_push_points - size(nodes,1),:);
for i = 1:size(new_pp,1)
    plot([new_pp(i,1), nodes(new_pp(i,3),1)], [new_pp(i,2), nodes(new_pp(i,3),2)], 'k:','LineWidth',2);
    %text(new_pp(i,1),new_pp(i,2),num2str(i+length(nodes)), 'FontSize', 24);
end
plot(push_points(possible_push_points - size(nodes,1),1),push_points(possible_push_points - size(nodes,1),2),'*b','MarkerSize',24)
new_nodes = nodes;
new_nodes([18,14,10,28],:) = [];
for i = 1:length(new_nodes)
plot(new_nodes(i,1),new_nodes(i,2), 'r*','MarkerSize',24);
%text(new_nodes(i,1),new_nodes(i,2),num2str(i), 'FontSize', 14);
end






%image 4: Updated VG and PG
figure(11); hold on;
plotCurrentState(Map, curr_node, nodes, 3, 11)
old_VG = VG(1:size(nodes,1),1:size(nodes,1));
plotVisibilityGraph(old_VG, nodes, 'b', 'c', 7);
plotPushabilityGraph(PG, nodes, 'r', 'g--' , 3);
curr_pos = nodes(curr_node.vehicle_pos,:);
vehicle_circle = circlePoints(curr_pos, Map.disc_radius, 25);
plot(vehicle_circle(:,1),vehicle_circle(:,2),'r', 'LineWidth',3)
new_nodes = nodes;
new_nodes([18,14,10,28],:) = [];
for i = 1:length(new_nodes)
plot(new_nodes(i,1),new_nodes(i,2), 'r*','MarkerSize',24);
%text(new_nodes(i,1),new_nodes(i,2),num2str(i), 'FontSize', 14);
end