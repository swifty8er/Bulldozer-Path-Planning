function T = createTranspositionTable(num_of_nodes)
    num_of_bits = 32;
    % Create Zorbist Key
    T.Zorbist = createZorbistKey(num_of_nodes, num_of_bits );
    %set up the transposition table
    T.size = bitshift(1,18)+9; %try 23 if not opimal
    empty_node.cf = 0;
    empty_node.key = 0;
    empty_node.visited = false;
    node_cell_array{1} = empty_node;
    for i = 1:T.size
        T.Table(i) = node_cell_array;
    end
return;
end