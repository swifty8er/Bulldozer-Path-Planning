%find the shortest perpendicular distance between a list of nodes and a
%line
function dist = shortestNodeDistance(Map, nodes, line, from, to)
    dist = inf;
    for i = 1:length(Map.Obstacles)
        nodes = [nodes; Map.Obstacles{i}];
    end
    for i = 1:length(nodes)
        if ((i ~= from) && (i ~= to))
            curr_dist = point2LineDist(line, nodes(i,:));
            if (dist > curr_dist)
                dist = curr_dist;
            end
        end
    end
return;
end