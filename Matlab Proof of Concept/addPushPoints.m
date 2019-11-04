%adds the push points to the VG. These are appended to the end of the
%matrix with the start push points coming first then the destination push
%points. 
function [VG, Cond_Vertices] = addPushPoints(VG, push_dest_points, nodes, Map, Cond_Vertices)
    %add rows for the push points
    VG = [VG, zeros(size(nodes,1), size(push_dest_points,1))];
    VG = [VG; zeros(size(push_dest_points,1), size(VG,2))];
    %add the boundary to the Map obstacles
    Map.Obstacles{end+1} = Map.boundary;
    for i = 1:size(push_dest_points,1)
        for j = 1:size(nodes,1) %add +i-1  if you want conncetions between different push and dest points
            intersect = false;
            %find the line between the push point and the node or other
            %push point
            Line1.Point1_x = push_dest_points(i,1);
            Line1.Point1_y = push_dest_points(i,2);
            from = 0;
            if (j <= size(nodes,1))
                Line1.Point2_x = nodes(j,1);
                Line1.Point2_y = nodes(j,2);
                to = j;
            else
                Line1.Point2_x = push_dest_points(j-size(nodes,1),1);
                Line1.Point2_y = push_dest_points(j-size(nodes,1),2);
                to = 0;
            end
            %check if an obstacle node is too close to a line and which
            %nodes are too close tolerance should be a bit less than Map.disc_radius
            [close_nodes, close_obstacle] = findCloseNodes(Map, nodes, Line1, from, to, 2*Map.disc_radius); 
            if (close_obstacle == true) 
                intersect = true;
            end
            %check that push points from the same node cannot connect
            %to each other
            if (((j <= size(nodes,1)) || (push_dest_points(i,3) ~= push_dest_points(j-size(nodes,1),3))) && (push_dest_points(i,3) ~= j))
                %check every obstacle against the connection between two nodes
                k = 1;
                while (k <= length(Map.Obstacles)) && (intersect == false)
                    curr_obs = cell2mat(Map.Obstacles(k));
                    l = 1;
                    %check if any line of an obstacle intersects the line
                    %between two nodes
                    while (l < length(curr_obs)) && (intersect == false)
                        Line2.Point1_x = curr_obs(l,1);
                        Line2.Point1_y = curr_obs(l,2);
                        Line2.Point2_x = curr_obs(l+1,1);
                        Line2.Point2_y = curr_obs(l+1,2);
                        intersect = LinesIntersect(Line1, Line2);
                        l = l + 1;
                    end
                    k = k + 1;
                end
            else
                intersect = true;
            end
            if (intersect == true)
                VG(i+size(nodes,1),j) = 0;
                VG(j,i+size(nodes,1)) = 0;
            else
                VG(i+size(nodes,1),j) = 1;
                VG(j,i+size(nodes,1)) = 1;
                if (~isempty(close_nodes))
                    cond_vertex.i = i+size(nodes,1);
                    cond_vertex.j = j;
                    cond_vertex.nodes = close_nodes;
                    Cond_Vertices = [Cond_Vertices,cond_vertex];
                end
            end
        end
    end


return;
end