import numpy as np
import matplotlib.pyplot as plt
import random
import math

import pandas as pd

# As checked from Robile3



class Line:
    """An object used to create a line from two points

    :param start: First point used to generate the line. It's an
                  array of form [x,y].
    :type start: numpy.ndarray
    :param end: Second point used to generate the line. It's an
                array of form [x,y].
    :type end: numpy.ndarray
    """
    def __init__(self, start: np.ndarray, end: np.ndarray):
        if np.shape(start)!= (2,):
            raise ValueError("Start point must have the shape (2,)")
        if np.shape(end) != (2,):
            raise ValueError("End point must have the shape (2,)")
        if (start==end).all():
            raise ValueError("Start and end points must be different")
        
        # Calculate useful properties of the line
        self.start = start
        self.line = end - start
        self.length = np.linalg.norm(self.line)
        self.unit_line = self.line / self.length
        
    def point_dist(self, point: np.ndarray):
        """Calculate the distance between a point and a line segment.
        adapted from code given by clued__init__ found here:
        https://www.py4u.net/discuss/186227

        To calculate the closest distance to a line, we calculate
        the orthogonal distance from the point to the line.

        :param point: Numpy array of form [x,y], describing the point.
        :type point: numpy.ndarray
        :return: The minimum distance to a point.
        :rtype: float
        """
        if np.shape(point)!= (2,):
            raise ValueError("Start point must have the shape (2,)")
        # compute the perpendicular distance to the theoretical infinite line
        return np.linalg.norm(np.cross(self.line, self.start - point)) /self.length
    
    def equation(self):
        """Calculate the basic linear equation parameters useful for plotting

        :return: (m, c) where m is the gradient of the line
                 and c is the y intercept
        :rtype: tuple of floats
        """
        m = self.line[1]/self.line[0]
        c = self.start[1] - m*self.start[0]
        return (m, c)

def np_polar2rect(np_array):
    center = np.array([0,0])
    r = np_array.T[0,]
    theta = np_array.T[1,]
    x = r*np.sin(np.deg2rad(theta))
    y = r*np.cos(np.deg2rad(theta))
    return np.array([x, y]).T


def np_polar2cart(polar_points):
    center = np.array([0,0])
    r = polar_points.T[0,]
    theta = polar_points.T[1,]
    x = r*np.cos(theta) # theta -> radians
    y = r*np.sin(theta)
    return np.array([x, y]).T    

def np_cart2polar(cart_points):
    """
    cart_points: n x 2 matrix of points in cartesian coordinates
    """
    r = np.linalg.norm(cart_points, axis=1)
    theta = np.arctan2(cart_points[:,1], cart_points[:,0])
    return np.array([r, theta]).T   

        

def reduction_filter(data_points, sigma, k, median_filter_size, mean_filter_size):
    '''
    Method to reduce the number of laser scan points by first applying a median filter,
    then a mean filter, and finally performing the reduction by replacing a cluster of points
    with their representative.
    :param data_points: 2D array representing laser scan data in cartesian coordinates
    :param type: np.ndarray
    :param sigma: maximum distance between two consecutive points to consider them 
    in the same cluster
    :param type: float    
    :param k: maximum number of points allowed in a cluster
    :param type: int
    :param median_filter_size: size of the median filter window
    :param type: int
    :param mean_filter_size: size of the mean filter window
    :param type: int
    '''
    points_list = list(data_points)
    output = []

    # Apply median filter
    if median_filter_size > 1:
        median_filtered = []
        for i in range(len(output)):
            median_window = output[max(0, i - median_filter_size + 1):i + 1]
            median_filtered.append(np.median(np.array(median_window), axis=0))
        output = median_filtered
    
    # Apply reduction filter
    while len(points_list) > 0:
        a_i = points_list.pop(0)
        cur_sum = np.array(a_i)
        i = 1
    
        while len(points_list) > 0 and abs(a_i[0] - points_list[0][0]) < sigma and i <= k:
            cur_sum += np.array(points_list.pop(0))
            i += 1
        
        output.append(cur_sum / i)
        
    # Apply mean filter
    if mean_filter_size > 1:
        mean_filtered = []
        for i in range(len(output)):
            mean_window = output[max(0, i - mean_filter_size + 1):i + 1]
            mean_filtered.append(np.mean(np.array(mean_window), axis=0))
        output = mean_filtered
        
    return np.array(output)

def online_line_detection(points_array, e=0.01, incr=0.02, max_dist=2, k=25):
    """
    arguments:
    points  : ordered list of arrays of individual points
    e       : allowed fraction of deviation of sum of distances between consecutive points and 
    the total length  of the line
    incr    : increment in the error values with the number of points
    max_dist: maximum distance between consecutive points allowed in a line segment
    k       : minimum number of points required in a line segment
    return: return a list of numpy array containing points grouped into points on the same line
    """
    points_polar = np_cart2polar(points_array)
    points_polar_sorted = points_polar[points_polar[:, 1].argsort()]
    points_cart = np_polar2cart(points_polar_sorted)
    points = [point for point in points_cart]

    grouped_points = []
    d_m_c_endPts = []           # line parameters: perpendicular distance, slope and deviation from laser scanner
    point = points[0]
    current_group = [point]
    aj = point                  # starting point of current line segment
    dist_sum = 0                # sum of distances between consecutive points o a line segment
    ak = point                  # latest point added to a line
    ek = e                      # incremented error

    for point_idx in range(1,len(points)): 
        dist_sum += np.linalg.norm(ak-points[point_idx])
        full_dist_ratio = np.linalg.norm(aj-points[point_idx])/dist_sum
        end_dist_ratio = 1 
        
        if len(current_group) >= k:
            prev_ak = points[point_idx-2]
            end_dist_ratio = (np.linalg.norm(prev_ak-points[point_idx])/ (np.linalg.norm(prev_ak-ak)+np.linalg.norm(ak-points[point_idx])))
        
        if(full_dist_ratio > 1-ek and end_dist_ratio > 1-e and np.linalg.norm(ak-points[point_idx]) <= max_dist):
            current_group.append(points[point_idx])
            ak = points[point_idx]
            ek += incr

        else:
            if len(current_group) >= k:
                grouped_points.append(np.array(current_group))
                
            current_group = [points[point_idx]]
            aj = points[point_idx]
            dist_sum = 0
            ak = points[point_idx]
            ek = e
            
    if len(current_group) >= k:
        grouped_points.append(np.array(current_group))

    if grouped_points:
        for points_on_line in grouped_points:
            line_segment = Line(points_on_line[0], points_on_line[-1])
            end_pts = np.array([points_on_line[0], points_on_line[-1]])
            m, c = line_segment.equation()
            center_wrt_laser = np.array([-0.45, 0])       # location of center of robot with respect to laser scanner
            d = line_segment.point_dist(center_wrt_laser) # distance fom the center of the Robile
            d_m_c_endPts.append([d, m, c, end_pts])      
    return d_m_c_endPts

def plot_lines(lines):
    plt.figure(figsize=(6, 6))
    for line_num, line_params in enumerate(lines):
        _, m, c, end_points = line_params
        plt.plot(end_points[:,0], end_points[:,1], label=f'Line {line_num + 1}')

    plt.grid()
    plt.title("Online Line Detection for: ")
    plt.xlabel("x position")
    plt.ylabel("y position")
    plt.legend()
    plt.show()
