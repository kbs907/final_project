#! /usr/bin/env python
# -*- coding: utf-8 -*-

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


    def detect_first_obstacle(self):
        for i in range((15*505)/360, (45*505)/360):
            if self.lidar_points[i] <= 0.3 and self.lidar_points[i] > 0.0:
                return 'left'
        for i in range((315*505)/360, (345*505)/360):
            if self.lidar_points[i] <= 0.3 and self.lidar_points[i] > 0.0:
                return 'right'
        return None

    def detect_obstacle_lidar_flag(self, first_obstacle, flag):
        if flag == False
            if first_obstacle == 'left' :
                if self.lidar_points[(90*505)/360] < 0.4 and self.lidar_points[(90*505)/360] > 0.0:
                    return True
            elif first_obstacle =='right' :
                if self.lidar_points[(270*505)/360] < 0.4 and self.lidar_points[(270*505)/360] > 0.0:
                    return True
        if flag == True
            if first_obstacle == 'left' and (self.lidar_points[(90*505)/360] > 0.45 or self.lidar_points[(90*505)/360] == 0.0):
                return False
            if first_obstacle == 'right' and (self.lidar_points[(270*505)/360] > 0.45 or self.lidar_points[(270*505)/360] == 0.0):
                return False

    def detect_obstacle_left(self): #왼쪽에 장애물 있을 때
        #f_lidar = np.concatenate((self.lidar_points[:(30*505)/360], self.lidar_points[(330*505)/360:]))
        f_lidar = self.lidar_points[:(30*505)/360] + self.lidar_points[(330*505)/360:]
        for i in f_lidar:
            if i <= 0.3 and i > 0.0:
                return 'right'
        for i in range((60*505)/360, (90*505)/360):
            if self.lidar_points[i] <= 0.3 and self.lidar_points[i] > 0.0:
                return 'left'

    def detect_obstacle_right(self):  #오른쪽 장애물 있을 때
        #f_lidar = np.concatenate((self.lidar_points[:(30*505)/360],self.lidar_points[(330*505)/360:]))
        f_lidar = self.lidar_points[:(30*505)/360] + self.lidar_points[(330*505)/360:]
        for i in f_lidar:
            if i <= 0.3 and i > 0.0:
                return 'left'

        for i in range((270*505)/360, (300*505)/360):
            if self.lidar_points[i] <= 0.3 and self.lidar_points[i] > 0.0:
                return 'right'
