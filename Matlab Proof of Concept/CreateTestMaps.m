function TestMaps = CreateTestMaps()

%Get Microban Level Data
clear
fid = fopen('Microban Levels.txt','r');
%data = textscan(fid,"%s", 'Delimiter', '\n');
data = fread(fid,'*char')';
fclose(fid);
raw_Mb = splitlines(data);

%find indices of all level titles
ii = find(contains(raw_Mb, 'Level') == 1);
%where the next level title would be if there was one
ii(end+1) = length(raw_Mb) + 1;
Microban_GridMaps{1} = [''];

%Get a cell array of each Microban Map in string form
for q = 1:length(ii)-1
    %build the string that builds the map
    M = '';
    for r = ii(q)+1:ii(q+1)-2
        M = join([M,char(raw_Mb(r)),':']);
    end

    row_length = 0;
    longest_length = 0; %longest row length
    col_length = 0;
    %find the column and maximum row length
    for i = 1:length(M)
        %test if it's a new line
        if (M(i) == ':')
            if (longest_length < row_length)
                longest_length = row_length;
            end
            col_length = col_length + 1;
            row_length = 0;
        else
            row_length = row_length + 1;
        end
    end
    col_length = col_length + 1;

    GridMap = '';
    GridMap(1:col_length,1:longest_length) = '#';
    i = 1;
    j = 1;
    wall_flag = false;
    %Put the map in string form into a 2d character array
    for k = 1:length(M)
        %test if it's a new line
        if (M(k) == ':')
            i = i + 1;
            j = 1;
            wall_flag = false;
        else
            %check if you have seen a wall yet
            %this to avoid other characters appearing outside the walls of
            %the map
            if (wall_flag == false)
                if (M(k) == '#')
                    wall_flag = true;
                end
            else
                GridMap(i,j) = M(k);
            end
            j = j + 1;
        end
    end
    
    Microban_GridMaps{q} = GridMap;

end

Outlines{length(Microban_GridMaps)} = [];
Obstacles{length(Microban_GridMaps)} = {};
%Convert string form maps into cartesian coordinate maps
for i = 1:length(Microban_GridMaps)
    %Map out the lines that make up the boundary
    curr_gridmap = char(Microban_GridMaps{i});
    %Find the starting point for the boundary
    [start_i, start_j] = find(curr_gridmap ~= '#',1);

    %Trace the boundary
    first_node = [start_i, start_j];
    %find first curr node after first step
    path = first_node;
    curr_node = [start_i, start_j];
    new_node = [0,0];
    S = curr_node;
    first = true;
    curr_ori = [-1,0];
    %Do a wall hug algorithm to find the outline, only search NESW in that order
    while ((first == true) || ((first_node(1) ~= curr_node(1)) || (first_node(2) ~= curr_node(2))))
        %[curr_node, S] = pop(S);
        %find which adjacent nodes are valid
        left_ori = turnLeft(curr_ori);
        curr_left = curr_node + left_ori;
        curr_forward = curr_node + curr_ori;
        [max_i, max_j] = size(curr_gridmap);
        %if there is a wall on the left and the check index is valid
        if ((ifIndexIsValid(curr_left, max_i, max_j)) && ...
                (curr_gridmap(curr_left(1),curr_left(2)) == '#'))
            %if there is no wall in front and check index is valid
            if ((ifIndexIsValid(curr_forward, max_i, max_j)) && ...
                    (curr_gridmap(curr_forward(1),curr_forward(2)) ~= '#'))
                %move forward
                curr_node = curr_forward;
                path = [path;curr_node];
                first = false;
            else
                %turn right on the spot
                curr_ori = turnRight(curr_ori);
            end
        else
            %turn left
            curr_node = curr_left;
            path = [path;curr_node];
            curr_ori = left_ori;
            first = false;
        end          
    end
    Outlines{i} = path;
    %Convert GridMap into Binary Image
    BI = zeros(size(curr_gridmap));
    ii = find(curr_gridmap == '#');
    BI(ii) = 1;
    %Find any obstacles inside the boundary
    obs_struct = bwconncomp(BI,4);
    curr_obslist = obs_struct.PixelIdxList;
    r = 1;
    %find the boundary obstacle
    while (r <= length(curr_obslist))
        curr_obs = curr_obslist{r};
        if (~isempty(find(curr_obs == 1, 1)))
            curr_obslist(r) = [];
        end
        r = r + 1;
    end
    
    %Now find the outlines of the obstalces
    %Find the starting point for the boundary
    if (~isempty(curr_obslist))
        empty_obslist = {};
        empty_obslist{length(curr_obslist)} = [];
        Obstacles{i} = empty_obslist;
    end
    for q = 1:length(curr_obslist)
        if (length(curr_obslist{q}) == 1)
            %obstacle is just a box
            [start_i, start_j] = ind2sub(size(curr_gridmap),curr_obslist{q}(1));
            curr_outlines = Outlines{i};
            max_y = max(curr_outlines(:,1));
            min_x = min(curr_outlines(:,2));
            curr_obs_outline = [start_j-min_x,max_y+1-start_i];
            curr_obs_outline(end+1,:) = curr_obs_outline(end,:) + [1,0];
            curr_obs_outline(end+1,:) = curr_obs_outline(end,:) + [0,-1];
            curr_obs_outline(end+1,:) = curr_obs_outline(end,:) + [-1,0];
            curr_obs_outline(end+1,:) = curr_obs_outline(end,:) + [0,1];
            Obstacles{i}{q} = curr_obs_outline;
        else
            [start_i, start_j] = ind2sub(size(curr_gridmap),curr_obslist{q}(1));

            %Trace the boundary of the obstacles
            curr_outlines = Outlines{i};
            max_y = max(curr_outlines(:,1));
            min_x = min(curr_outlines(:,2));
            curr_obs_outline = [start_j-min_x,max_y+1-start_i];
            first_node = [start_i, start_j];
            %find first curr node after first step
            curr_node = [start_i, start_j];
            first = true;
            visited = first_node;
            curr_ori = [-1,0];
            num_obs_blocks = length(curr_obslist{q});
            k = 1;
            %Do a wall hug algorithm to find the obstacle outline, only search NESW in that order
            while ((num_obs_blocks ~= size(visited,1)) || ((first_node(1) ~= curr_node(1)) || (first_node(2) ~= curr_node(2))))
                %[curr_node, S] = pop(S);
                %find which adjacent nodes are valid
                left_ori = turnLeft(curr_ori);
                curr_left = curr_node + left_ori;
                curr_forward = curr_node + curr_ori;
                [max_i, max_j] = size(curr_gridmap);
                %if there is no wall on the left and check index is valid
                if ((ifIndexIsValid(curr_left, max_i, max_j)) && ...
                        (curr_gridmap(curr_left(1),curr_left(2)) ~= '#'))
                    %if there is a wall in front and check index is valid
                    if ((ifIndexIsValid(curr_forward, max_i, max_j)) && ...
                            (curr_gridmap(curr_forward(1),curr_forward(2)) == '#'))
                        %move forward
                        curr_node = curr_forward;
                        first = false;
                        if (IsIndexInPath(visited, curr_node))
                            visited = [visited;curr_node];
                        end
                        %convert to cartesian cooridinate direction
                        xy_direction = fliplr(curr_ori);
                        xy_direction(2) =  -xy_direction(2);

                        curr_obs_outline(k+1,:) = curr_obs_outline(k,:) + xy_direction;
                        k = k + 1;
                    else
                        %turn right on the spot
                        curr_ori = turnRight(curr_ori);
                        %convert to cartesian cooridinate direction
                        xy_direction = fliplr(curr_ori);
                        xy_direction(2) =  -xy_direction(2);

                        curr_obs_outline(k+1,:) = curr_obs_outline(k,:) + xy_direction;
                        k = k + 1;
                    end
                else
                    %turn left
                    curr_node = curr_left;
                    curr_ori = left_ori;
                    first = false;
                    if (IsIndexInPath(visited, curr_node))
                        visited = [visited;curr_node];
                    end
                    %convert to cartesian cooridinate direction
                    xy_direction = fliplr(curr_ori);
                    xy_direction(2) =  -xy_direction(2);

                    curr_obs_outline(k,:) = [];
                    k = k - 1;
                    curr_obs_outline(k+1,:) = curr_obs_outline(k,:) + xy_direction;
                    k = k + 1;
                end          
            end
        end
        if ((curr_obs_outline(1,1) ~=  curr_obs_outline(end,1)) || (curr_obs_outline(1,2) ~=  curr_obs_outline(end,2)))
            curr_obs_outline(end+1,:) = curr_obs_outline(1,:);
        end
        Obstacles{i}{q} = curr_obs_outline;
    end
end

% Plot a grid map and its path
% image_map = zeros(size(curr_gridmap));
% image_map(curr_gridmap == '@') = 1;
% image_map(curr_gridmap == '.') = 2;
% image_map(curr_gridmap == '$') = 3;
% image_map(curr_gridmap == '*') = 4;
% image_map(curr_gridmap == ' ') = 5;
% figure(1)
% imagesc(image_map)
% figure(2)
% path_map = zeros(size(curr_gridmap));
% for i = 1:size(path,1)
%     path_map(path(i,1),path(i,2)) = path_map(path(i,1),path(i,2)) + 1;
%     imagesc(path_map)
%     pause(0.5)
% end

%Trace the boundary
%two grid spaces if theres a orientation change. first one is always two
Boundaries{length(Outlines)} = [];
for i =  1:length(Outlines)
    curr_outlines = Outlines{i};
    max_y = max(curr_outlines(:,1));
    curr_boundary = [0,max_y+1-curr_outlines(1,1)];
    curr_ori = [-1,0];
    k = 1;
    for j = 1:size(curr_outlines,1)-1
        direction = curr_outlines(j+1,:) - curr_outlines(j,:);
        left_dir = turnLeft(curr_ori);
        right_dir = turnRight(curr_ori);
        %move forward
        if ((direction(1) == curr_ori(1)) && (direction(2) == curr_ori(2)))
            %convert to cartesian cooridinate direction
            xy_direction = fliplr(direction);
            xy_direction(2) =  -xy_direction(2);
            
            curr_boundary(k+1,:) = curr_boundary(k,:) + xy_direction;
            k = k + 1;
        %left turn
        elseif ((direction(1) == left_dir(1)) && (direction(2) == left_dir(2)))
            %convert to cartesian cooridinate direction
            xy_direction = fliplr(direction);
            xy_direction(2) =  -xy_direction(2);
            
            curr_boundary(k,:) = [];
            k = k - 1;
            curr_boundary(k+1,:) = curr_boundary(k,:) + xy_direction;
            k = k + 1;
            curr_ori = direction;
        %right turn
        elseif ((direction(1) == right_dir(1)) && (direction(2) == right_dir(2)))
            %convert to cartesian cooridinate direction
            xy_direction = fliplr(direction);
            xy_direction(2) =  -xy_direction(2);
            
            curr_boundary(k+1,:) = curr_boundary(k,:) + xy_direction;
            k = k + 1;
            curr_boundary(k+1,:) = curr_boundary(k,:) + xy_direction;
            k = k + 1;
            curr_ori = direction;
        %180 turn
        else
            %convert to cartesian cooridinate direction
            xy_direction = fliplr(direction);
            xy_direction(2) =  -xy_direction(2);
            xy_right_dir = fliplr(right_dir);
            xy_right_dir(2) =  -xy_right_dir(2);
            
            curr_boundary(k+1,:) = curr_boundary(k,:) + xy_right_dir;
            k = k + 1;
            curr_boundary(k+1,:) = curr_boundary(k,:) + xy_direction;
            k = k + 1;
            curr_boundary(k+1,:) = curr_boundary(k,:) + xy_direction;
            k = k + 1;
            curr_ori = direction;
        end
    end
    if ((curr_boundary(1,1) ~=  curr_boundary(end,1)) || (curr_boundary(1,2) ~=  curr_boundary(end,2)))
        curr_boundary(end+1,:) = curr_boundary(1,:);
    end
    Boundaries{i} = curr_boundary;
end

%plot boundaries and obstacles
% for i = 1:length(Boundaries)
%     figure(2)
%     clf
%     hold on
%     curr_boundary = Boundaries{i};
%     plot(curr_boundary(:,1), curr_boundary(:,2), 'r');
%     curr_obslist = Obstacles{i};
%     for j = 1:length(curr_obslist)
%     curr_obs = curr_obslist{j};
%         if (~isempty(curr_obs))
%             plot(curr_obs(:,1), curr_obs(:,2), 'b');
%         end
%     end
%     disp(i)
%     pause(1);
%     hold off
% end


for i = 1:length(Microban_GridMaps)
    curr_boundary = Boundaries{i};
    curr_Map.number = i;
    curr_Map.min_x = min(curr_boundary(:,1));
    curr_Map.max_x = max(curr_boundary(:,1));
    curr_Map.min_y = min(curr_boundary(:,2));
    curr_Map.max_y = max(curr_boundary(:,2));
    curr_Map.grid_size = 0.025;
    curr_Map.boundary = curr_boundary;
    curr_Map.Obstacles = Obstacles{i};
    curr_Map.disc_radius = 0.45;
    curr_Map.vehicle_radius = 0.45;
    
    %find vehicle, disk and goal positions
    curr_gridmap = Microban_GridMaps{i};
    curr_outlines = Outlines{i};
    max_y = max(curr_outlines(:,1));
    min_x = min(curr_outlines(:,2));
    curr_obs_outline = [k-min_x,max_y+1-j];
    curr_Map.goal_pos_xy = [];
    curr_Map.initial_vehicle_pos_xy = [];
    curr_Map.initial_disk_pos_xy = [];
    for j = 1:size(curr_gridmap,1)
        for k = 1:size(curr_gridmap,2)
            if (curr_gridmap(j,k) == '.')
                %goal
                curr_Map.goal_pos_xy(end+1,:) = [k-min_x,max_y+1-j]; %converted
                %move to centre
                curr_Map.goal_pos_xy(end,:) = curr_Map.goal_pos_xy(end,:) + [0.5,-0.5];
            elseif (curr_gridmap(j,k) == '@')
                %vehicle
                curr_Map.initial_vehicle_pos_xy = [k-min_x,max_y+1-j]; %converted
                %move to centre
                curr_Map.initial_vehicle_pos_xy(end,:) = curr_Map.initial_vehicle_pos_xy(end,:) + [0.5,-0.5];
            elseif (curr_gridmap(j,k) == '$')
                %disk
                curr_Map.initial_disk_pos_xy(end+1,:) = [k-min_x,max_y+1-j]; %converted
                %move to centre
                curr_Map.initial_disk_pos_xy(end,:) = curr_Map.initial_disk_pos_xy(end,:) + [0.5,-0.5];
            elseif (curr_gridmap(j,k) == '*')
                %disk on top of goal
                curr_Map.initial_disk_pos_xy(end+1,:) = [k-min_x,max_y+1-j]; %converted
                curr_Map.goal_pos_xy(end+1,:) = [k-min_x,max_y+1-j]; %converted
                %move to centre
                curr_Map.initial_disk_pos_xy(end,:) = curr_Map.initial_disk_pos_xy(end,:) + [0.5,-0.5];
                curr_Map.goal_pos_xy(end,:) = curr_Map.goal_pos_xy(end,:) + [0.5,-0.5];
            elseif (curr_gridmap(j,k) == '+')
                %vehicle on top of goal
                curr_Map.initial_vehicle_pos_xy(end+1,:) = [k-min_x,max_y+1-j]; %converted
                curr_Map.goal_pos_xy(end+1,:) = [k-min_x,max_y+1-j]; %converted
                %move to centre
                curr_Map.initial_vehicle_pos_xy(end,:) = curr_Map.initial_vehicle_pos_xy(end,:) + [0.5,-0.5];
                curr_Map.goal_pos_xy(end,:) = curr_Map.goal_pos_xy(end,:) + [0.5,-0.5];
            end
        end
    end
    
    TestMaps{i} = curr_Map;
end

% for i = 1:length(TestMaps)
%     figure(3);
%     clf
%     curr_Map = TestMaps{i};
%     axis([curr_Map.min_x, curr_Map.max_x,  curr_Map.min_y, curr_Map.max_y]);
%     hold on;
%     plot(curr_Map.boundary(:,1), curr_Map.boundary(:,2), 'k');
%     for j = 1:length(curr_Map.Obstacles)
%         curr_obs = curr_Map.Obstacles{j};
%         plot(curr_obs(:,1), curr_obs(:,2), 'k');
%     end
%     plot(curr_Map.initial_vehicle_pos_xy(1),curr_Map.initial_vehicle_pos_xy(2),'ro','MarkerSize',30)
%     plot(curr_Map.initial_disk_pos_xy(:,1),curr_Map.initial_disk_pos_xy(:,2),'bo','MarkerSize',30)
%     plot(curr_Map.goal_pos_xy(:,1),curr_Map.goal_pos_xy(:,2),'go','MarkerSize',35)
%     disp(i)
%     pause(1)
%     hold off
% end

return;
end

function [node, S] = pop(S)
    if (~isempty(S))
        node = S(end,:);
        S(end,:) = [];
    else
        node = [];
        S = [];
    end
return;
end

function S = push(node, S)
    S(end+1,:) = node;
return;
end

function inPath = IsIndexInPath(path, index)
    inPath = false;
    i = 1;
    while ((i <= size(path,1)) && (inPath == false))
        if ((path(i,1) == index(1)) && (path(i,2) == index(2)))
            inPath = true;
        end
        i = i + 1;
    end
return;
end

function hasAdjaWall = gridSpaceNextToWall(Map, index)
    directions = [-1,0;-1,1;0,1;1,1;1,0;1,-1;0,-1;-1,-1];
    i = 1;
    hasAdjaWall = false;
    while ((i <= length(directions)) && (hasAdjaWall == false))
        new_index = index + directions(i,:);
        if (Map(new_index) == '#')
            hasAdjaWall = true;
        end
        i = i + 1;
    end
return;
end

function ori = turnLeft(ori)
    if (ori(1) ~= 0)
        ori(2) = ori(1);
        ori(1) = 0;
    else
        ori(1) = -ori(2);
        ori(2) = 0;
    end
return;
end

function ori = turnRight(ori)
    if (ori(1) ~= 0)
        ori(2) = -ori(1);
        ori(1) = 0;
    else
        ori(1) = ori(2);
        ori(2) = 0;
    end
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