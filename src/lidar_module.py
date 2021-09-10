#! /usr/bin/env python

class LidarModule :
    def __init__(self):
        self.lidar_points = None

    def set_lidardata(self, data):
        self.lidar_points = data.ranges

    def can_rotary_in(self):
        if len([i for i in self.lidar_points[42:84] if i>0 and i<1.5]) :
            return False
        return True

    def forward_obstacle(self):
        if len([i for i in self.lidar_points[-63:]+self.lidar_points[:63] if i>0 and i<0.3]) :
            return True
        return False
