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
from nav_msgs.msg      import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from random import randint

sys.path.insert(0, '/home/kpochana/robotws/src/me169/scripts/util')
from prmtools import *

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
GRID_X = -3.8100
GRID_Y = -3.8100
GRID_THETA = 0.0

NPOINTS = 400



class SimplePlanner:

    def __init__(self): 
        self.get_map()
        self.path = []
        self.markermsg = Marker()
        self.points = []
        self.generated = False
        self.prmstates = []
        rospy.Subscriber("/pose", PoseStamped, self.cb_pose)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.cb_goal)
        rospy.Subscriber("/scan", LaserScan, self.cb_laser)      
        self.waypoints_pub = rospy.Publisher("/waypoints", Marker, queue_size = NPOINTS)
        self.line_pub = rospy.Publisher("/line", Marker)
        # rospy.Subscriber("/wheel_state", JointState, self.grab_theta)
        self.pub = rospy.Publisher('/vel_cmd', Twist, queue_size=10)

        self.generate_PRM(NPOINTS)
        self.atgoal = True
        self.direction = ''
        self.pos = np.zeros(3)
        self.goal = np.zeros(3)
        self.object = False
        

    def get_map(self):
        # Wait 30sec for a map.
        rospy.loginfo("Waiting for a map...")
        self.h = 300
        self.w = 300
        self.mapmsg = rospy.wait_for_message('/map', OccupancyGrid, 30.0)
        self.map = np.array(self.mapmsg.data).reshape(self.h, self.w)
        self.resolution = self.mapmsg.info.resolution

    def generate_PRM(self, npoints):
        count = 0

        self.markermsg.header.frame_id = "map"
        self.markermsg.header.stamp = rospy.Time.now()
        self.markermsg.ns = "waypoints"
        self.markermsg.id = 0
        self.markermsg.type = Marker.POINTS
        self.markermsg.action = Marker.ADD
        self.markermsg.pose.position.x = 0.0
        self.markermsg.pose.position.y = 0.0
        self.markermsg.pose.position.z = 0.0
        self.markermsg.pose.orientation.x = 0.0
        self.markermsg.pose.orientation.y = 0.0
        self.markermsg.pose.orientation.z = 0.0
        self.markermsg.pose.orientation.w = 1.0
        self.markermsg.scale.x = 0.05
        self.markermsg.scale.y = 0.05
        self.markermsg.scale.z = 0.05
        self.markermsg.color.a = 1.0
        self.markermsg.color.r = 0.0
        self.markermsg.color.g = 1.0
        self.markermsg.color.b = 0.0
        self.markermsg.type = 7
        self.markermsg.points = []

        self.prmstates.append(State(0.0, 0.0, 0.0))

        while count < npoints:
            u = randint(0, self.w - 1)
            v = randint(0, self.h - 1)

            if self.map[v, u] == 0:
                count += 1
                self.points.append([u, v])
                x = float(u + 0.5) * self.resolution + GRID_X
                y = float(v + 0.5) * self.resolution + GRID_Y
                self.markermsg.points.append(Point(x, y, 0.0))
                self.prmstates.append(State(x, y, 0.0)) # x y theta

        print(self.markermsg.points)

        self.waypoints_pub.publish(self.markermsg)
        self.generated = True



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
        # print(self.object)
        # average /= len(ranges)
        # print(average)
        # print("++++++++++++++++++++++++++++++++")

    def grab_theta(self, msg):
        self.pos[2] = msg.position[3]

    def plan(self, p, q):
        self.prmstates.append(State(p.x, p.y, 0.0))
        self.nodes = []
        self.nodes.append(Node(State(self.pos[0], self.pos[1], self.get_theta(q))))
        for state in self.prmstates:
            self.nodes.append(Node(state))
        
        print("Graph contains", len(self.nodes), "nodes")
        
        self.path = AStar(self.nodes, self.nodes[0], self.nodes[-1])
        if not self.path:
            print("\n-----\n\nPATH NOT FOUND\n\n-----\n")
            return

        self.linemsg = Marker()
        self.linemsg.header.frame_id = "map"
        self.linemsg.header.stamp = rospy.Time.now()
        self.linemsg.ns = "waypoints"
        self.linemsg.id = 0
        self.linemsg.type = Marker.POINTS
        self.linemsg.action = Marker.ADD
        self.linemsg.pose.position.x = 0.0
        self.linemsg.pose.position.y = 0.0
        self.linemsg.pose.position.z = 0.0
        self.linemsg.pose.orientation.x = 0.0
        self.linemsg.pose.orientation.y = 0.0
        self.linemsg.pose.orientation.z = 0.0
        self.linemsg.pose.orientation.w = 1.0
        self.linemsg.scale.x = 0.05
        self.linemsg.scale.y = 0.05
        self.linemsg.scale.z = 0.05
        self.linemsg.color.a = 1.0
        self.linemsg.color.r = 1.0
        self.linemsg.color.g = 0.0
        self.linemsg.color.b = 0.0
        self.linemsg.type = Marker.LINE_STRIP
        self.linemsg.points = []
        for node in self.path:
            #print(Point(node.state.x, node.state.y, 0.0))
            self.linemsg.points.append(Point(node.state.x, node.state.y, 0.0))

        self.goal = [self.path[0].state.x, self.path[0].state.y, 0.0]
        self.line_pub.publish(self.linemsg)

    def cb_goal(self, msg):
        assert(msg.header.frame_id == 'map')
        p = msg.pose.position
        q = msg.pose.orientation
        # self.goal = np.array([p.x, p.y, self.get_theta(q)])
        self.direction = ''
        self.atgoal = False
        print("Goal Received")
        self.plan(p, q)
        

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
        if self.path != None \
           and len(self.path) != 0 \
           and math.isclose(self.goal[0], self.pos[0], rel_tol=0.06) \
           and math.isclose(self.goal[1], self.pos[1], rel_tol=0.06):
            print('Next step...')
            self.path = self.path[1:]
            if len(self.path) > 1:
                self.goal = np.array([self.path[0].state.x, 
                                      self.path[0].state.y, 
                                      np.arctan2(self.path[1].state.y - self.path[0].state.y, self.path[1].state.x - self.path[0].state.x)])
            elif len(self.path) == 1:
                self.goal = np.array([self.path[0].state.x, 
                                      self.path[0].state.y, 
                                      self.path[0].state.theta])

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
                if not np.array_equal(self.goal, self.pos):
                    print("REACHED WAYPOINT!")
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

        if self.generated:
            self.waypoints_pub.publish(self.markermsg)


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

