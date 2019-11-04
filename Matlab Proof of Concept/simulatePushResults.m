%runs a basic visualisation of a push path planning solution
function simulatePushResults(vehicle_path, disk_path, all_nodes, push_point_rng, dest_points_rng, Map, fig_num, make_gif)
    plotMap(Map,fig_num);
    if (make_gif == true)
        %setup to capture as gif
        axis tight manual % this ensures that getframe() returns a consistent size
        filename = strcat('Gifs\Map_', int2str(Map.number), '_Animated.gif');
        h = figure(fig_num);
    end
    %run through all vehicle steps
    j = 2;
    goal_pos = all_nodes(Map.goal_pos,:);
    for i = 1:size(goal_pos,1)
        goal_circle = circlePoints(goal_pos(i,:), Map.disc_radius*1.1, 25);
        plot(goal_circle(:,1),goal_circle(:,2),'g');
    end
    curr_pos = all_nodes(vehicle_path(1),:);
    curr_disk_pos = [];
    for i = 1:length(disk_path)
        curr_disk_path = disk_path{i};
        curr_disk_pos = [curr_disk_pos; all_nodes(curr_disk_path(1),:)];
    end
    curr_disk_index = ones(length(disk_path),1);
    vehicle_circle = circlePoints(curr_pos, Map.disc_radius, 25);
    Sim.vehicle_handle = plot(vehicle_circle(:,1),vehicle_circle(:,2),'r');
    for i = 1:size(curr_disk_pos,1)
        disk_circle = circlePoints(curr_disk_pos(i,:), Map.disc_radius, 25);
        Sim.disk_handle(i) = plot(disk_circle(:,1),disk_circle(:,2),'b');
    end
    for i = 2:length(vehicle_path)
        %get and plot vehicle position
        curr_pos = all_nodes(vehicle_path(i),:);
        vehicle_circle = circlePoints(curr_pos, Map.disc_radius, 25);
        set(Sim.vehicle_handle,'xdata',vehicle_circle(:,1),'ydata',vehicle_circle(:,2));
        %find transition in vehicle path when vehicle pushes a disk
        if ((vehicle_path(i-1) <= push_point_rng(2)) && (vehicle_path(i-1) >= push_point_rng(1)) && ...
            (vehicle_path(i) <= dest_points_rng(2)) && (vehicle_path(i) >= dest_points_rng(1)))
            %find closest disk to the vehicle
            closest_disk = findClosestDisk(all_nodes(vehicle_path(i-1),:), disk_path, curr_disk_index, all_nodes);
            %store the next move to be made by the disks if valid
            curr_disk_path = disk_path{closest_disk};
            if (length(curr_disk_path) >= curr_disk_index(closest_disk) + 1)
                curr_disk_index(closest_disk) = curr_disk_index(closest_disk) + 1;
                curr_disk_pos(closest_disk,:) = all_nodes(curr_disk_path(curr_disk_index(closest_disk)),:);
            end
            %plot new disk position
            for k = 1:size(curr_disk_pos,1)
                disk_circle = circlePoints(curr_disk_pos(k,:), Map.disc_radius, 25);
                set(Sim.disk_handle(k),'xdata',disk_circle(:,1),'ydata',disk_circle(:,2));
            end
            j = j + 1;
        end
        if (make_gif == true)
            % Capture the plot as an image 
            frame = getframe(h); 
            im = frame2im(frame); 
            [imind,cm] = rgb2ind(im,256); 
            % Write to the GIF File 
          if (i == 2) 
              imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
          else 
              imwrite(imind,cm,filename,'gif','WriteMode','append'); 
          end 
        end
        %w = waitforbuttonpress;
        %pause(0.5);
    end
return;
end

%find finds the closest disk to the curr vehicle position
function closest_disk = findClosestDisk(vehicle_pos, disk_path, curr_disk_index, nodes)
    closest_dist = inf;
    %check all disks
    for i = 1:length(disk_path)
        %find that particular disks current position
        curr_disk = disk_path{i};
        curr_disk_pos = nodes(curr_disk(curr_disk_index(i)), :);
        %find the distance between the vehicle and the disk
        curr_dist = sqrt((vehicle_pos(1) - curr_disk_pos(1))^2+(vehicle_pos(2) - curr_disk_pos(2))^2);
        if (curr_dist < closest_dist)
            closest_dist = curr_dist;
            closest_disk = i;
        end
    end
return;
end

function circle = circlePoints(centre, radius, numOfPoints)
    circle = zeros(numOfPoints,2);
    for i = 1:numOfPoints
        circle(i,1) = centre(1) + radius*cos((i-1)*(2*pi/(numOfPoints-1)));
        circle(i,2) = centre(2) + radius*sin((i-1)*(2*pi/(numOfPoints-1)));
    end
return;
end