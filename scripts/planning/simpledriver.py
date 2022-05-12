#!/usr/bin/env python3
#
#   simpledriver.py
#
#

import math
import sys
import time
import rospy
import smbus
import numpy as np

from sensor_msgs.msg import JointState, LaserScan
from nav_msgs.msg      import Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist

VEL_MAX = 0.2
OMEGA_MAX = 6
LAMBDA = 1.0
TOLERANCE = 0.05
GAMMA = 0.5
ANGLE_MAX = 0.7743043303489685
ANGLE_MIN = -0.760712206363678
ANGLE_INCREMENT = 0.0036203220952302217
ANGLE_RANGE = 0.436
RANGE_MAX = 4.0
RANGE_MIN = 0.05
RANGE_CUTOFF = 0.2



class SimplePlanner:

    def __init__(self):    
        rospy.Subscriber("/pose", PoseStamped, self.cb_pose)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.cb_goal)
        rospy.Subscriber("/scan", LaserScan, self.cb_laser)        
        # rospy.Subscriber("/wheel_state", JointState, self.grab_theta)
        self.pub = rospy.Publisher('/vel_cmd', Twist, queue_size=10)
        self.atgoal = True
        self.direction = ''
        self.pos = np.zeros(3)
        self.goal = np.zeros(3)
        self.object = False

    def cb_laser(self, msg):
        ranges = msg.ranges
        self.object = False
        curr_angle = ANGLE_MIN
        # average = 0.0
        for dist in ranges:
            if (curr_angle > -ANGLE_RANGE and curr_angle < ANGLE_RANGE):
                if (dist <= RANGE_CUTOFF and dist >= RANGE_MIN):
                    self.object = True
                    break
                    # print("dist: ", dist)
                    # print("angle: ", curr_angle)
                    
            # average += dist
            curr_angle += ANGLE_INCREMENT
        print(self.object)
        # average /= len(ranges)
        # print(average)
        print("++++++++++++++++++++++++++++++++")

    def grab_theta(self, msg):
        self.pos[2] = msg.position[3]

    def cb_goal(self, msg):
        assert(msg.header.frame_id == 'map')
        p = msg.pose.position
        q = msg.pose.orientation
        self.goal = np.array([p.x, p.y, self.get_theta(q)])
        self.direction = ''
        self.atgoal = False
        print("Goal Received")

    def choose_dir(self, theta1, theta2):
        theta = 0.0
        if abs(theta2 - theta1) > abs(theta1 - theta2):
            theta = theta1 - theta2
        else:
            theta = theta2 - theta1
        if theta > np.pi:
            theta -= 2.0 * np.pi
        elif theta < -1 * np.pi:
            theta += 2.0 * np.pi
        return theta

# \actual_exposure\
    def cb_pose(self, msg):
        assert(msg.header.frame_id == 'map')
        # p = msg.pose.pose.position
        # q = msg.pose.pose.orientation
        p = msg.pose.position
        q = msg.pose.orientation
        self.pos[0:2] = np.array([p.x, p.y])
        self.pos[2] = self.get_theta(q) 

        dir = self.goal[0:2] - self.pos[0:2] 
        dir_theta = np.arctan2(dir[1], dir[0])
        # if (self.direction == '' and dir_theta < 0):
        #     self.direction = 'l'
        # elif self.direction == '':
        #     self.direction = 'r'

        # if (self.direction == 'l' and dir_theta > 0):
        #     dir_theta -= 2 * np.pi
        # elif (self.direction == 'r' and dir_theta < 0):
        #     dir_theta += 2 * np.pi

        dist = np.linalg.norm(dir)
        v_x = 0.0
        w_z = 0.0

        if (dist >= 0.05 and not np.array_equal(self.goal, self.pos)):
            angle = self.choose_dir(dir_theta, self.pos[2])
            w_z = -LAMBDA * angle
            v_x = min(VEL_MAX, GAMMA * dist) * np.cos(angle)
            # print("STEP ONE: ", dist, "\t", angle)
        else:
            angle = self.choose_dir(self.goal[2], self.pos[2])

            w_z = -LAMBDA * angle

            if (np.abs(angle) < 0.05):
                self.goal = self.pos
            # print("STEP TWO: ", dist, "\t", angle)
        
        # This should save the goal and continue to move there if the object is out of the way
        # Needs testing
        if (self.object):
            if v_x > 0:
                v_x = 0.0

        msg = Twist()
        msg.linear.x = v_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = w_z
        self.pub.publish(msg)


    def get_theta(self, q):
        x = 2.0 * np.arctan2(q.z, q.w)
        return x

    


if __name__ == "__main__":
    
    rospy.init_node('planner')

    planner = SimplePlanner()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Planner spinning...")
    rospy.spin()
    rospy.loginfo("Planner stopped.")

