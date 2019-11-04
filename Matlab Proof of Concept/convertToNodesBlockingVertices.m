%Go thorugh every element in the array Cond_Vertices and find every node
%that disables a vertex and sort the new array into a list of elements that
%that inform you on which nodes disable which vertices where the index is
%the node number and the element is a 2d array of the vertices'
%cooridinates in the VG
function NodesBlockVertices = convertToNodesBlockingVertices(Cond_Vertices, numOfNodes)
    
    initial_struct.vertices = [];
    NodesBlockVertices(numOfNodes) = initial_struct;
    for j = 1:length(Cond_Vertices)
        for k = 1:length(Cond_Vertices(j).nodes)
            ii = Cond_Vertices(j).nodes(k);
            NodesBlockVertices(ii).vertices(end+1,1) = Cond_Vertices(j).i;
            NodesBlockVertices(ii).vertices(end,2) = Cond_Vertices(j).j;
        end
    end

return;
end