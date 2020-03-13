import copy

class PQNode():
    #All info required to keep track of each state
    # At the moment using greedy search, f = h (no g)
    def __init__(self, vehicle_pos, vehicle_dest_pt, vehicle_decision_path, disk_poses, disk_num, travelled, vehicle_path, disk_path, dist2goal):
        if len(vehicle_path) > 0 and vehicle_dest_pt >= 0 and len(vehicle_decision_path) > 0:
            node_vehicle_path = vehicle_path.copy()
            for point in vehicle_decision_path:
                node_vehicle_path.append(point)
            node_vehicle_path.append(vehicle_dest_pt)
        else:
            node_vehicle_path = [vehicle_pos]

        if disk_num >= 0 and len(disk_path) > 0:
            node_disk_path = copy.deepcopy(disk_path)
            node_disk_path[disk_num].append(disk_poses[disk_num])
        else:
            node_disk_path = []
            for disk in disk_poses:
                node_disk_path.append([disk])

        self._cf = travelled + dist2goal
        self._travelled = travelled
        self._vehicle_pos = vehicle_pos
        self._vehicle_path = node_vehicle_path
        self._disk_poses = disk_poses.copy()
        self._disk_path = node_disk_path

    
    #def __getitem__(self, index):
    #    return {
    #            0: self._cf,
    #            1: self._travelled,
    #            2: self._vehicle_pos,
    #            3: self._vehicle_path,
    #            4: self._disk_poses,
    #            5: self._disk_path
    #            }[index]

    def __lt__(self,pq_node):
        return self._cf < pq_node.cf

    @property
    def vehicle_pos(self):
        return self._vehicle_pos

    @property
    def disk_poses(self):
        return self._disk_poses

    @property
    def vehicle_path(self):
        return self._vehicle_path

    @property
    def disk_path(self):
        return self._disk_path

    @property
    def cf(self):
        return self._cf

    def printNode(self):
        print("Vehicle Pos:", self._vehicle_pos)
        print("Vehicle Path:")
        for point in self._vehicle_path:
            print(point)
        print("Disk Poses:", self._disk_poses)
        for disk_point in self._disk_path:
            print(disk_point)
        print("Travelled:", self._travelled)
        print("Cost:", self._cf)
        
        
        
        
        