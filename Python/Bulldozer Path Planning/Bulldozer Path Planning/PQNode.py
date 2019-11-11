class PQNode():
    #All info required to keep track of each state
    def __init__(self, vehicle_pos, vehicle_dest_pt, disk_pos, disk_num, travelled, vehicle_path, disk_path, dist2goal):
        #(self, vehicle_pos, vehicle_path, disk_pos, disk_path, travelled, cf):
        if vehicle_dest_pt >= 0:
            if len(vehicle_path) > 0:
                vehicle_path.append(vehicle_dest_pt)
            else:
                vehicle_path = [vehicle_dest_pt]

        if vehicle_dest_pt >= 0:
            curr_disk_path = disk_path[disk_num]
            if len(curr_disk_path) > 0:
                curr_disk_path.append(disk_pos[disk_num])
            else:
                curr_disk_path = [disk_pos[disk_num]]
            disk_path[disk_num] = curr_disk_path

        self._cf = travelled + dist2goal
        self._travelled = travelled
        self._vehicle_pos = vehicle_pos
        self._vehicle_path = vehicle_path
        self._disk_pos = disk_pos
        self._disk_path = disk_path

        #self._cf = cf
        #self._travelled = travelled
        #self._vehicle_pos = vehicle_pos
        #self._vehicle_path = vehicle_path
        #self._disk_pos = disk_pos
        #self._disk_path = disk_path
        
        

    
    def __getitem__(self, index):
        return {
                0: self._cf,
                1: self._travelled,
                2: self._vehicle_pos,
                3: self._vehicle_path,
                4: self._disk_pos,
                5: self._disk_path
                }[index]


#    #initial state
#curr_node.vehicle_pos = Map.initial_vehicle_pos##
#curr_node.disk_pos = Map.initial_disk_pos###
#curr_node.vehicle_path = Map.initial_vehicle_pos###
#for i = 1:length(curr_node.disk_pos)
#    curr_node.disk_path{i} = Map.initial_disk_pos(i)###
#end
#curr_node.travelled = 0
##calculate cost function
#min_cf(1:length(curr_node.disk_pos)) = inf
#for m = 1:length(curr_node.disk_pos)
#    for n = 1:length(Map.goal_pos)
#        disk_pos = points_and_nodes(curr_node.disk_pos(m),:)
#        goal_pos = points_and_nodes(Map.goal_pos(n),:)
#        curr_cf = sqrt((disk_pos(1)-goal_pos(1))^2+(disk_pos(2)-goal_pos(2))^2)+curr_node.travelled
#        if (curr_cf < min_cf(m))
#            min_cf(m) = curr_cf
#        end
#    end
#end
#curr_node.cf = sum(min_cf)