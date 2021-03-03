'''
    Adapted from course 16831 (Statistical Techniques).
    Initially written by Paloma Sodhi (psodhi@cs.cmu.edu), 2018
    Updated by Wei Dong (weidong@andrew.cmu.edu), 2021
'''

import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm

from map_reader import MapReader


class SensorModel:
    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """
    def __init__(self, occupancy_map):
        """
        TODO : Tune Sensor Model parameters here
        The original numbers are for reference but HAVE TO be tuned.
        """
        self._z_hit = 10
        self._z_short = 0.1
        self._z_max = 0.1
        self._z_rand = 10000
        
        z_sum = self._z_hit + self._z_short + self._z_max + self._z_rand 
        self._z_hit = self._z_hit / z_sum
        self._z_short = self._z_short / z_sum
        self._z_max = self._z_max / z_sum
        self._z_rand = self._z_rand / z_sum

        self._sigma_hit = 50
        self._lambda_short = 0.1

        self._max_range = 1000
        # self._max_range = 8184
        self._min_probability = 0.35
        self.bool_occ_map = (occupancy_map < self._min_probability) & (occupancy_map >= 0)
        
        # self.stored_occ_map = occupancy_map
#         self._subsampling = 2

#         self._norm_wts = 1.0

    def ray_cast(self, laser_x_t1, laser_y_t1, laser_theta, walk_stride):
        
        # calc_distance = 0
        # x_index = math.floor(laser_x_t1 / 10)
        # y_index = math.floor(laser_y_t1 / 10)

        # if self.bool_occ_map[y_index][x_index]==False:
        #     return 1

        # while x_index >= 0 and  x_index <= 799 and y_index >= 0 and  y_index <= 799:
        #     laser_x_t1 = laser_x_t1 + walk_stride * math.cos(laser_theta)
        #     laser_y_t1 = laser_y_t1 + walk_stride * math.sin(laser_theta)

        #     x_index = math.floor(laser_x_t1 / 10)
        #     y_index = math.floor(laser_y_t1 / 10)

        #     if self.bool_occ_map[y_index][x_index]==False:
        #         calc_distance += walk_stride / 2
        #         break
        #     else:
        #         calc_distance += walk_stride

        # if calc_distance > self._max_range:
        #     calc_distance = self._max_range # this is when the wall is further than max_range distance away
        x_end = laser_x_t1
        y_end = laser_y_t1
        x_idx = int(np.around(x_end/10))
        y_idx = int(np.around(y_end/10))

        temp_location = self.bool_occ_map[y_idx][x_idx]
        while x_idx >= 0 and  x_idx <= 799 and y_idx >= 0 and  y_idx <= 799 and temp_location==True:
            temp_location = self.bool_occ_map[y_idx][x_idx]
            x_end += walk_stride * np.cos(laser_theta)
            y_end += walk_stride * np.sin(laser_theta)
            x_idx = int(np.around(x_end/10))
            y_idx = int(np.around(y_end/10))

        calc_distance = math.sqrt((laser_x_t1-x_end)**2+(laser_y_t1-y_end)**2)
        calc_distance = calc_distance-walk_stride/2
        return calc_distance 

    def calc_prob(self, ray_cast_distance, measurement_distance):
        prob_sum = 0
        
        if measurement_distance >= 0:

        #p_hit
        
            coef_norm = 1/ (self._sigma_hit * math.sqrt(2*math.pi)) 
            exp_term = math.exp((-1/2)* ((ray_cast_distance - measurement_distance)**2/self._sigma_hit**2))
            p_hit = coef_norm * exp_term
            prob_sum +=  self._z_hit * p_hit

        #p_short
        
            if measurement_distance >= 0 and measurement_distance <= ray_cast_distance:
                normalizer = 1/(1-math.exp(-self._lambda_short*ray_cast_distance))
                p_short = normalizer*self._lambda_short*math.exp(-self._lambda_short*measurement_distance)
            else:
                p_short = 0

            prob_sum += self._z_short * p_short

        #p_max

            if measurement_distance <= self._max_range and measurement_distance >= self._max_range - 5: #NEED TO TUNE THE WIDTH OF THIS
                p_max = 1.0
            else:
                p_max = 0.0

            prob_sum += self._z_max * p_max

        #p_rand

            if measurement_distance < self._max_range:
                p_rand = 1/(self._max_range)
            else:
                p_rand = 0.0

            prob_sum += self._z_rand * p_rand
        if prob_sum == 0:
            print("PROB_SUM = 0")
            print("ray_cast_distance: ", ray_cast_distance)
            print("measurement_distance: ", measurement_distance)
        
        return prob_sum
    
    
    def beam_range_finder_model(self, z_t1_arr, x_t1,table):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """
        """
        TODO : Add your code here
        """
        walk_stride = 8
        angle_stride = 10
        
        # Assume that we have distance metric for every k from ray casting
        x,y,theta = x_t1

        laser_x_t1 = x + 25 * math.cos(theta)
        laser_y_t1 = y + 25 * math.sin(theta)
        # print("laser_x_t1:",laser_x_t1)
        prob_zt1 = 1.0
        #CAN CHANGE ANGLE_STRIDE!!!
        for deg in range (-90, 90, angle_stride):
            
            
            laser_theta = (deg + theta * 180 / math.pi ) * (math.pi/180)
            measurement_distance = z_t1_arr[deg+90]

            # ray_cast_distance = self.ray_cast_read(laser_x_t1, laser_y_t1, laser_theta, table)
            ray_cast_distance= self.ray_cast(laser_x_t1, laser_y_t1, laser_theta, walk_stride)
            
            # print("laser theta",laser_theta*180/math.pi)
            # print("ray cast", ray_cast_distance)
            # print(" ")

            particle_prob = self.calc_prob(ray_cast_distance, measurement_distance)
            prob_zt1 *= particle_prob

        robot_x_index = math.floor(x / 10)
        robot_y_index = math.floor(y / 10)

        if self.bool_occ_map[robot_y_index][robot_x_index]==False:
            prob_zt1 = 0
            
        return prob_zt1

    def ray_cast_read(self,laser_x_t1, laser_y_t1, laser_theta, table):

        output = 1
        if(laser_x_t1 < 3000 or laser_x_t1 >= 7000 or laser_y_t1<0 or laser_y_t1 >= 8000):
            return output
            
        xlow = int(laser_x_t1/5)*5
        xhigh =int(xlow if xlow == 6995 else xlow + 5) 
        ylow = int(laser_y_t1/5)*5
        yhigh = int(ylow if xlow == 7995 else ylow + 5)
        laser_theta = laser_theta*180/math.pi if laser_theta>=0 else laser_theta*180/math.pi+180*2

        thetalow = min(int(laser_theta/5),71)
        thetahigh = int(thetalow if thetalow == 355 else thetalow + 5)

        if (xhigh-xlow) ==0: 
            xd = 0
        if (yhigh-ylow) ==0: 
            yd = 0
        if (thetahigh-thetalow) ==0: 
            zd = 0
        xd = (laser_x_t1-xlow)/(xhigh-xlow)
        yd = (laser_y_t1-ylow)/(yhigh-ylow)
        zd = (laser_theta-thetalow)/(thetahigh-thetalow)

        xlow_idx = int((xlow-3000)/5)
        xhigh_idx = int((xhigh-3000)/5) 
        ylow_idx = int(ylow/5)
        yhigh_idx = int(yhigh/5) 
        thetalow_idx = int(thetalow/2)
        thetahigh_idx = int(thetahigh/2)

        c000 = table[xlow_idx][ylow_idx][thetalow_idx]
        c100 = table[xhigh_idx][ylow_idx][thetalow_idx]
        c010 = table[xlow_idx][yhigh_idx][thetalow_idx]
        c110 = table[xhigh_idx][yhigh_idx][thetalow_idx]
        c001 = table[xlow_idx][ylow_idx][thetahigh_idx]
        c101 = table[xhigh_idx][ylow_idx][thetahigh_idx]
        c011 = table[xlow_idx][yhigh_idx][thetahigh_idx]
        c111 = table[xhigh_idx][yhigh_idx][thetahigh_idx]

        c00 = c000*(1-xd) + c100*xd
        c01 = c001*(1-xd) + c101*xd
        c10 = c010*(1-xd) + c110*xd
        c11 = c011*(1-xd) + c111*xd

        c0 = c00*(1-yd) + c10*yd
        c1 = c01*(1-yd) + c11*yd

        output = c0*(1-zd)+ c1*zd
        return output
        

