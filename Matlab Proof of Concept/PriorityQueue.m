function [node, PQ] = pop(PQ)
    if (~isempty(PQ))
        node = PQ(1);
        PQ(1) = [];
    else
        node = [];
        PQ = [];
    end
return;
end

function PQ = push(PQ, point, travelled, path, goal)
    path(end+1,:) = point;
    node.point = point;
    node.cf = travelled + goalDistance(point, goal);
    node.path = path;
    node.travelled = travelled;
    PQ(length(PQ) + 1) = node;
return;
end


function PQ = sortQueue(PQ)
    for i = 1:length(PQ)-1
        for j = 1:length(PQ)-i
            if (PQ(j).cf > PQ(j+1).cf)
                tmp = PQ(j);
                PQ(j) = PQ(j+1);
                PQ(j+1) = tmp;
            end
        end
    end
return;
end