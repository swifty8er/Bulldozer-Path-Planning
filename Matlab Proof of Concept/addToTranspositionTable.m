%adds a node to the transposition table
%Status:
%   E = table was empty for that state
%   R = values existed but were replaced (node value was better)
%   N = values existed and weren't replaced (table value was better)
function [T, status] = addToTranspositionTable(node, T)
    % Get the Zorbist Key for a state
    key = getZorbistKey(node, T.Zorbist);
    index = mod(key,T.size)+1;
    
    %check if a value already exists
    if (T.Table{index}(1).key == 0)
        T.Table{index}(1).key = key;
        T.Table{index}(1).cf = node.cf;
        status = 'E';
    else
        %check if current node is the same as stored node(s)
        i = 1;
        found = false;
        while ((i <= length(T.Table{index})) && (found == false))
            if (key == T.Table{index}(i).key)
                found = true;
                if (node.cf < T.Table{index}(i).cf)
                    T.Table{index}(i).cf = node.cf;
                    status = 'R';
                else
                    status = 'N';
                end
            end
            i = i + 1;
        end
        if (found == false)
            new_node.key = key;
            new_node.cf = node.cf;
            new_node.visited = false;
            T.Table{index}(end+1) = new_node;
            status = 'E';
        end
    end
return;
end