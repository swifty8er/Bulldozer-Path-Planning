class PQNode():
    #All info required to keep track of each state
    def __init__(self, vehicle_pos, vehicle_dest_pt, vehicle_decision_path, disk_poses, disk_num, travelled, vehicle_path, disk_path, dist2goal):
        if len(vehicle_path) > 0 and vehicle_dest_pt >= 0 and len(vehicle_decision_path) > 0:
            for point in vehicle_decision_path:
                vehicle_path.append(point)
            vehicle_path.append(vehicle_dest_pt)
        else:
            vehicle_path = [vehicle_pos]

        if disk_num >= 0 and len(disk_path) > 0:
            disk_path[disk_num].append(disk_poses[disk_num])
        else:
            disk_path = []
            for disk in disk_poses:
                disk_path.append([disk])

        self._cf = travelled + dist2goal
        self._travelled = travelled
        self._vehicle_pos = vehicle_pos
        self._vehicle_path = vehicle_path
        self._disk_poses = disk_poses
        self._disk_path = disk_path

    
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