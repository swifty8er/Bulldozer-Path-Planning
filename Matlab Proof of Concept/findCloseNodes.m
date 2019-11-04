%find the shortest perpendicular distance between a list of nodes and a
%line
function [close_nodes, close_obstacle] = findCloseNodes(Map, nodes, line, from, to, tolerance)
    close_nodes = [];
    i = 1;
    close_obstacle = false;
    %see if any obstacle nodes are too close to the line
    while ((i <= length(Map.Obstacles)) && (close_obstacle == false))
        curr_obs_node = Map.Obstacles{i};
        j = 1;
        while ((j <= length(curr_obs_node)) && (close_obstacle == false))
            curr_dist = point2LineDist(line, curr_obs_node(j,:)); %should be minimum distance between two lines
            if (curr_dist < tolerance/2)
                close_obstacle = true;
            end
            j = j + 1;
        end
        i = i +1;
    end
    %see all obstalces are clear then check to see which nodes are to close
    %to the line
    if (close_obstacle == false)
        for i = 1:length(nodes)
            if ((i ~= to) && (i ~= from)) 
                curr_dist = point2LineDist(line, nodes(i,:));
                if ((tolerance-curr_dist) > eps(100)) %determine if curr dist is less than tolerance
                    close_nodes = [close_nodes, i];
                end
            end
        end
    end
return;
end
