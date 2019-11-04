%Converts a matrix VG into an array of cells. Each cell contains a list of
%nodes that node can see
function CellVG = VGtoCellArray(VG)
    for i = 1:size(VG,1)
        curr_cell = [];
        for j = 1:size(VG,2)
            if (VG(i,j) ~= 0)
                curr_cell = [curr_cell, j];
            end
        end
        CellVG{i} = curr_cell;
    end
return;
end