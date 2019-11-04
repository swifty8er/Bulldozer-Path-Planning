class Map:
    """description of class"""

    def __init__(self, num, minx, miny, maxx, maxy, gridsize, bound, obs, diskradi, vehradi, goalpos, vehpos, diskpos):
        self._number = num
        self._min_x = minx
        self._max_x = maxx
        self._min_y = miny
        self._max_y = maxy
        self._grid_size = gridsize
        self._boundary = bound
        self._obstacles = obs
        self._disk_radius = diskradi
        self._vehicle_radius = vehradi
        self._goal_pos_xy = goalpos
        self._initial_vehicle_pos_xy = vehpos
        self._initial_disk_pos_xy = diskpos
        self._goal_pos = []
        self._initial_vehicle_pos = 0
        self._initial_disk_pos = []

    @property
    def number(self):
        return self._number

    @property
    def min_x(self):
        return self._min_x

    @property
    def max_x(self):
        return self._max_x

    @property
    def min_y(self):
        return self._min_y

    @property
    def max_y(self):
        return self._max_y

    @property
    def grid_size(self):
        return self._grid_size

    @property
    def boundary(self):
        return self._boundary

    @property
    def obstacles(self):
        return self._obstacles

    @property
    def disk_radius(self):
        return self._disk_radius

    @property
    def vehicle_radius(self):
        return self._vehicle_radius

    @property
    def goal_pos_xy(self):
        return self._goal_pos_xy

    @property
    def initial_vehicle_pos_xy(self):
        return self._initial_vehicle_pos_xy

    @property
    def initial_disk_pos_xy(self):
        return self._initial_disk_pos_xy

    @property
    def goal_pos(self):
        return self._goal_pos

    @goal_pos.setter
    def goal_pos(self, value):
        self._goal_pos = value

    @property
    def initial_vehicle_pos(self):
        return self._initial_vehicle_pos

    @initial_vehicle_pos.setter
    def initial_vehicle_pos(self, value):
        self._initial_vehicle_pos = value

    @property
    def initial_disk_pos(self):
        return self._initial_disk_pos

    @initial_disk_pos.setter
    def initial_disk_pos(self, value):
        self._initial_disk_pos = value

    #def setToNodes:
        