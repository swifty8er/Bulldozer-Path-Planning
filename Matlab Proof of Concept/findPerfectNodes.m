function nodes = findPerfectNodes(Map)
    yy = [Map.min_y+0.5:1:Map.max_y+0.5]';
    nodes = [];
    for i = 1:size(yy,1)
        new_nodes = [Map.min_x+0.5:1:Map.max_x+0.5]';
        new_nodes(:,2) = yy(i);
        nodes = [nodes; new_nodes];
    end
    %remove any outbounds points
    ii = [];
    for i = 1:size(nodes,1)
        [inbounds,onbound] = inpolygon(nodes(i,1), nodes(i,2), Map.boundary(:,1),Map.boundary(:,2));
        if ((inbounds == false) || (onbound == true))
            ii(end+1) = i;
        end
    end
    nodes(ii,:) = [];
return;
end