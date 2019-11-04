function [VG, Cond_Vertices] = createVisibilityGraph(nodes, Map)
    %initialise visibility graph
    VG = zeros(length(nodes));
    Cond_Vertices = struct('i', {}, 'j', {}, 'nodes', {});
    %add the boundary to the Map obstacles
    Map.Obstacles{end+1} = Map.boundary;
    for i = 1:length(nodes)-1
        for j = i+1:length(nodes)
            Line1.Point1_x = nodes(i,1);
            Line1.Point1_y = nodes(i,2);
            Line1.Point2_x = nodes(j,1);
            Line1.Point2_y = nodes(j,2);
            intersect = false;
            %check if an obstacle node is too close to a line and which
            %nodes are too close tolerance should be a bit less than Map.disc_radius
            [close_nodes, close_obstacle] = findCloseNodes(Map, nodes, Line1, i, j, 2*Map.disc_radius); 
            if (close_obstacle == true) 
                intersect = true;
            end
            %check every obstacle against the connection between two nodes
            k = 1;
            while (k <= length(Map.Obstacles)) && (intersect == false)
                %check if any line of an obstacle intersects the line
                %between two nodes
                curr_obs = cell2mat(Map.Obstacles(k));
                s = 1;
                while (s < length(curr_obs)) && (intersect == false)
                    Line2.Point1_x = curr_obs(s,1);
                    Line2.Point1_y = curr_obs(s,2);
                    Line2.Point2_x = curr_obs(s+1,1);
                    Line2.Point2_y = curr_obs(s+1,2);
                    intersect = LinesIntersect(Line1, Line2);
                    s = s + 1;
                end
                k = k + 1;
            end
            if (intersect == true)
                VG(i,j) = 0;
                VG(j,i) = 0;
            else
                VG(i,j) = 1;
                VG(j,i) = 1;
                if (~isempty(close_nodes))
                    cond_vertex.i = i;
                    cond_vertex.j = j;
                    cond_vertex.nodes = close_nodes;
                    Cond_Vertices = [Cond_Vertices,cond_vertex];
                end
            end
        end
    end

return;
end