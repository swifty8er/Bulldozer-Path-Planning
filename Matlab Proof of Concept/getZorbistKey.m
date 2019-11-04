function key = getZorbistKey(node, Z)

    key = 0;
    for i = 1:length(Z)
        %find the current value for a certain node
        if (node.vehicle_pos == i)
            curr_value = Z(i).vehicle;
        elseif (~isempty(find(node.disk_pos == i,1))) %2nd most costly line
            curr_value = Z(i).disk;
        else
            curr_value = Z(i).empty; %Most costly line
        end
        %add to current key value
        key = bitxor(key,curr_value);
    end

return;
end