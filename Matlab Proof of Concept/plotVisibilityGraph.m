function plotVisibilityGraph(VG, nodes, colour1, colour2, line_size)
    for i = 1:size(VG,1)
        for j = 1:size(VG,2)
            if (VG(i,j) == 2)
                plot([nodes(i,1),nodes(j,1)],[nodes(i,2),nodes(j,2)], colour1, 'LineWidth',line_size)
            elseif (VG(i,j) == 1)
                plot([nodes(i,1),nodes(j,1)],[nodes(i,2),nodes(j,2)], colour2, 'LineWidth',line_size)
            end
        end
    end