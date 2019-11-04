%finds the points that a vehicle must be at to make a valid push from
%one node to another. Takes these results and puts it into a Pushability
%Graph
function [PG, all_push_points] = createPushabilityGraph(VG, nodes, Map)
    all_push_points = [];
    PG = zeros(size(VG));
    visited = zeros(size(nodes,1),1);
    Q = Map.goal_pos;
    while (~isempty(Q))
        curr_node = Q(1);
        visited(curr_node) = 1;
        Q(1) = [];
        for i = 1:size(VG,1)
            %checks if node isn't the same and that they can see each other
            if ((VG(i,curr_node) ~= 0) && (curr_node ~= i))
                %finds and adds the push point if it exists
                push_point = pushPoint(nodes(i,:), nodes(curr_node,:), Map, i, curr_node);
                if (~isempty(push_point))
                    PG(i,curr_node) = VG(i,curr_node);
                    %add to queue if not in there already
                    if ((isempty(find(Q == i,1))) && (visited(i) == 0))
                        Q(end+1) = i;
                    end
                    if (~pointInList(push_point, all_push_points))
                        all_push_points = [all_push_points; push_point];
                    end
                end
            end
        end
    end

return;
end

%{
function [PG, all_push_points] = createPushabilityGraph(VG, nodes, Map)
    all_push_points = [];
    PG = VG;
    for i = 1:size(VG,1)
        for j = 1:size(VG,2)
            %checks if node isn't the same and that they can see each other
            if ((VG(i,j) ~= 0) && (i ~= j))
                %finds and adds the push point if it exists
                push_point = pushPoint(nodes(i,:), nodes(j,:), Map, i, j);
                if (~isempty(push_point))
                    PG(i,j) = VG(i,j);
                    if (~pointInList(push_point, all_push_points))
                        all_push_points = [all_push_points; push_point];
                    end
                else
                    PG(i,j) = 0;
                end
            end
        end
    end
return;
end
%}