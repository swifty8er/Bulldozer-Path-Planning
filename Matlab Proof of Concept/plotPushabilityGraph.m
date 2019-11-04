function plotPushabilityGraph(PG, nodes, colour1, colour2, line_size)
    for i = 1:size(PG,1)
        for j = 1:size(PG,2)
            if (PG(i,j) == 2)
                quiver(nodes(i,1),nodes(i,2),nodes(j,1)-nodes(i,1),nodes(j,2)-nodes(i,2), 0, colour1, 'LineWidth',line_size)
            elseif (PG(i,j) == 1)
                quiver(nodes(i,1),nodes(i,2),nodes(j,1)-nodes(i,1),nodes(j,2)-nodes(i,2), 0, colour2, 'LineWidth',line_size)
            end
        end
    end
return;
end