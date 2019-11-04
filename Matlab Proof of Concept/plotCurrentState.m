function plotCurrentState(Map, curr_node, nodes, line_width, fig_num)
    figure(fig_num);
    axis([Map.min_x, Map.max_x,  Map.min_y, Map.max_y]);
    hold on;
    plot(Map.boundary(:,1), Map.boundary(:,2), 'k');
    for i = 1:length(Map.Obstacles)
        curr_obs = Map.Obstacles{i};
        plot(curr_obs(:,1), curr_obs(:,2), 'k');
    end
    goal_pos = nodes(Map.goal_pos,:);
    for i = 1:size(goal_pos,1)
        goal_circle = circlePoints(goal_pos(i,:), Map.disc_radius*1.1, 25);
        plot(goal_circle(:,1),goal_circle(:,2),'g', 'LineWidth',line_width);
    end
    curr_pos = nodes(curr_node.vehicle_pos,:);
    curr_disk_pos = nodes(curr_node.disk_pos,:);
    vehicle_circle = circlePoints(curr_pos, Map.disc_radius, 25);
    plot(vehicle_circle(:,1),vehicle_circle(:,2),'r', 'LineWidth',line_width);
    for i = 1:size(curr_disk_pos,1)
        disk_circle = circlePoints(curr_disk_pos(i,:), Map.disc_radius, 25);
        plot(disk_circle(:,1),disk_circle(:,2),'b', 'LineWidth',line_width);
    end
return;
end