#!/usr/bin/env python
import math
import numpy as np
import json
import serial

import rospy
from nav_msgs.msg import *

class KalmanOdometry:
    def __init__(self):
      self.wheel_radius = 0.125 # radius of wheels
      self.wheel_separation = 0.71 # distance between wheels

      self.std_of_odom_xy =  0.01 * self.wheel_radius # possible error of odometry in x,y assuming 0.01degree error in rotation measurement (dx = d_theta*r)
      self.std_odom_heading = math.atan(2 * self.std_of_odom_xy / self.wheel_separation)  # possible error of odometry in heading considering worst case of measurement in x and y
      self.std_odom_Ux = 9999 # possible error of velocity in x
      self.std_odom_Uy = 9999 # possible error of velocity in y
      self.std_odom_Wz = 9999 # possible error of angular velocity in z

      self.std_plab_xy = 0.25 # possible error of plab in x,y
      self.std_plab_heading = 9999 # possible error of plab in heading

      self.A = np.eye(3) # State Transition Matrix 
      self.B = np.eye(3) # Control Input Matrix 
      self.H = np.eye(3)

      self.Q = np.array([[self.std_of_odom_xy**2, 0, 0], [0, self.std_of_odom_xy**2, 0], [0, 0, self.std_odom_heading**2]])
      self.R = np.array([[self.std_plab_xy**2, 0, 0], [0, self.std_plab_xy**2, 0], [0, 0, self.std_plab_heading**2]])
      self.cnt_odom = self.odom_x = self.odom_y = self.odom_head = 0
      self.plab_x = self.plab_y = 0
      self.pre_odom_x = self.pre_odom_y = self.pre_odom_head = 0
      self.delta_odom_x_pre  = self.delta_odom_y_pre = self.delta_odom_head  = 0
      self.head_temp = []
      self.odom_x_temp = []
      self.odom_y_temp = []

      self.kalman_pub = rospy.Publisher("kalman_odometry", Odometry ,queue_size=10)

      self.odomsub = rospy.Subscriber("agv1/odom", Odometry, self.odom_callback)
      self.plabsub = rospy.Subscriber("plab_odometry", Odometry, self.plab_callback)
      self.kalman_odometry()

    def odom_callback(self, msg):
      self.odom_x = msg.pose.pose.position.x
      self.odom_y = msg.pose.pose.position.y
      self.odom_qw = msg.pose.pose.orientation.w
      self.odom_qz = msg.pose.pose.orientation.z
      self.odom_head = math.atan2((2 * (self.odom_qw * self.odom_qz)),(1 - 2 * (self.odom_qz * self.odom_qz)))
      self.head_temp.append(self.odom_head)
      self.odom_x_temp.append(self.odom_x)
      self.odom_y_temp.append(self.odom_y)
      self.cnt_odom += 1 
      self.delta_odom_x_pre  = self.odom_x - self.pre_odom_x
      self.delta_odom_y_pre  = self.odom_y - self.pre_odom_y
      self.delta_odom_head  = self.odom_head - self.pre_odom_head
      self.pre_odom_x = self.odom_x
      self.pre_odom_y = self.odom_y
      self.pre_odom_head = self.odom_head      

    def plab_callback(self, msg):
      self.plab_x = msg.pose.pose.position.x
      self.plab_y = msg.pose.pose.position.y

    def kalman_odometry(self):
      x_c = np.array([[self.odom_x], [self.odom_y], [self.odom_head]])
      p_c = np.array([[100, 0, 0], [0, 100, 0], [0, 0, 100]])
      kalman_pos = np.array([[np.take(x_c,0),np.take(x_c,1),np.take(x_c,2)]])
      print("+++++++++++++++++++++++++++++++++++++++")
      U = np.array([[0], [0], [0]])
      x_m = np.array([[0], [0], [0]])
      while not rospy.is_shutdown():

        if ((abs(self.delta_odom_x_pre) > 0) or (abs(self.delta_odom_y_pre) > 0) or (abs(self.delta_odom_head) > 0)):
        
          self.delta_center = (self.delta_odom_x_pre**2 + self.delta_odom_y_pre**2)**0.5
          self.delta_odom_x = self.delta_center * math.cos(x_c[2])
          self.delta_odom_y = self.delta_center * math.sin(x_c[2])
          U = np.array([[self.delta_odom_x], [self.delta_odom_y], [self.delta_odom_head]])
          x_p = np.dot(self.A,x_c) + np.dot(self.B,U)
          p_p = np.dot(np.dot(self.A,p_c),self.A.T) + self.Q
          if x_p[2] > math.pi: # if kalman heading is above pi 
            x_p[2] =  x_p[2] - (2 * math.pi)
          elif  x_p[2] < -math.pi:
            x_p[2] =  x_p[2] + (2 * math.pi)          
          plab_head = x_p[2]
          std_plab_heading = 1000
        
          plab_x_center = self.plab_x - 0.4 * math.cos(x_c[2])
          plab_y_center = self.plab_y - 0.4 * math.sin(x_c[2])
          x_m = np.array([[plab_x_center],[plab_y_center],[plab_head]])
          k_g = np.dot(np.dot(p_p,self.H.T), np.linalg.inv(np.dot(np.dot(self.H,p_p),self.H.T) + self.R))   # Kalman Gain
          x_c = x_p + np.dot(k_g, (x_m - np.dot(self.H, x_p))) # state update
    #         print(x_c.T)
          p_c = np.dot((np.eye(3) - np.dot(k_g,self.H)), p_p) # updated state covarian

          if x_c[2] > math.pi: # if kalman heading is above pi 
            x_c[2] =  x_c[2] - (2 * math.pi)
          elif  x_c[2] < -math.pi:
            x_c[2] =  x_c[2] + (2 * math.pi)
          a = [[np.take(x_c,0),np.take(x_c,1),np.take(x_c,2)]]
          kalman_pos = np.append(a,kalman_pos,axis=0)
          print(kalman_pos)
          x_c = x_p
          p_c = p_p

        if self.cnt_odom > 150:
          if ((max(self.head_temp) - min(self.head_temp)) < (math.pi / 180)):
            first_point = np.array([[self.odom_x_temp[0]], [self.odom_y_temp[0]], [self.head_temp[0]]])
            last_point = np.array([[self.odom_x_temp[-1]], [self.odom_y_temp[-1]], [self.head_temp[-1]]])
            dist_temp = np.linalg.norm(last_point-first_point)
            if dist_temp > 0.4:
              mean_kalman_first = np.mean(kalman_pos[:20],axis=0)
              mean_kalman_last = np.mean(kalman_pos[100:],axis=0)
              plab_head = math.atan2((mean_kalman_last[1]-mean_kalman_first[1]),(mean_kalman_last[0],mean_kalman_first[0]))
              # heading arrangment before kalman correction step
              if x_p[2] > math.pi: # if kalman heading is above pi 
                x_p[2] =  x_p[2] - (2 * math.pi)
              elif  x_p[2] < -math.pi:
                x_p[2] =  x_p[2] + (2 * math.pi) 
              std_plab_heading = 1 # possible error of plab in heading              
          print("/////////////////////////////////////////////////////////////")
          self.head_temp = []
          self.odom_x_temp = []
          self.odom_y_temp = []
          kalman_pos = np.array([])
          kalman_pos = x_c.T       
          self.cnt_odom = 0


        



if __name__ == '__main__':
    rospy.init_node('kalman', anonymous=True)

    KalmanOdometry()
"""          

          
        if self.cnt_odom > 100:
          if ((max(self.head_temp) - min(self.head_temp)) < (math.pi / 180)):
            first_point = np.array([[self.odom_x_temp[0]], [self.odom_y_temp[0]], [self.head_temp[0]]])
            last_point = np.array([[self.odom_x_temp[-1]], [self.odom_y_temp[-1]], [self.head_temp[-1]]])
            dist_temp = np.linalg.norm(last_point-first_point)
          self.cnt_odom = 0
          self.head_temp = []
          self.odom_x_temp = []
          self.odom_y_temp = []
          kalman_pos = np.array([])
          kalman_pos = x_c.T






"""          