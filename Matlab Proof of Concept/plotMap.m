function plotMap(Map, fig_num)
    figure(fig_num);
    axis([Map.min_x, Map.max_x,  Map.min_y, Map.max_y]);
    hold on;
    plot(Map.boundary(:,1), Map.boundary(:,2), 'k');
    for i = 1:length(Map.Obstacles)
        curr_obs = Map.Obstacles{i};
        plot(curr_obs(:,1), curr_obs(:,2), 'k');
    end
return;
end