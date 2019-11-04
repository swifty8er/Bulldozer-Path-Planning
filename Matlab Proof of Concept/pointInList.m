%tests if a point exists in an array
function exists = pointInList(point, array)
    exists = false;
    i = 1;
    while ((i <= size(array,1)) && (exists == false))
        j = 1;
        same_element = true;
        while ((j <= size(array,2)) && (same_element == true))
            if (array(i, j) ~= point(j))
                same_element = false;
            end
            j = j + 1;
        end
        if (same_element == true)
            exists = true;
        end
        i = i + 1;
    end

return;
end