#!/usr/bin/env python3
#
#   simpledriver.py
#
#

import math, sys, time, rospy, smbus
import numpy as np

from sensor_msgs.msg import JointState, LaserScan
from nav_msgs.msg      import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from random import randint

sys.path.insert(0, '/home/kpochana/robotws/src/me169/scripts/util')
from prmtools import *

VEL_MAX = 0.1
OMEGA_MAX = 0.6
OMEGA_MIN = 0.1
LAMBDA = 1.0
LAMBDA_DRIVING = 0.50
TOLERANCE = 0.05
GAMMA = 0.5
ANGLE_MAX = 0.7743043303489685
ANGLE_MIN = -0.760712206363678
ANGLE_STOP = 0.2
ANGLE_INCREMENT = 0.0036203220952302217
ANGLE_RANGE = 0.5
RANGE_MAX = 4.0
RANGE_MIN = 0.08
RANGE_CUTOFF = 0.4
GRID_X = -3.8100
GRID_Y = -3.8100
GRID_THETA = 0.0
RES = 0.0254
DIST_BLUR = 0.2
OBSTACLE_THRESH = 1
LASER_THRESH = 10
D_OBJECT = 0.2
BOT_WIDTH = 0.15


NPOINTS = 150

TSP_POINTS = np.array([[0.124, 2.70, 0.0], 
                       [-1.377, 2.776, 0.0], 
                       [-2.677, -0.429, 0.0], 
                       [2.295, -0.455, 0.0]])



class SimplePlanner:

    def __init__(self): 
        self.valid_path = True
        self.obstacle_dic = {}
        self.obstacles_added = 0
        self.badpath = False
        self.laser_dic = {}
        self.vx = 0.0
        self.wz = 0.0
        self.planning = False
        self.goal_final = None
        self.get_map()
        self.path = []
        self.markermsg = Marker()
        self.points = []
        self.generated = False
        self.prmstates = []
        self.wheel_control = 3
        rospy.Subscriber("/pose", PoseStamped, self.cb_pose)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.cb_goal)
        rospy.Subscriber("/scan", LaserScan, self.cb_laser)   
        rospy.Subscriber("/localizing", Bool, self.cb_localizing) 
        rospy.Subscriber("/obstacles", Marker, self.cb_obstacle)      
        self.waypoints_pub = rospy.Publisher("/waypoints", Marker, queue_size = NPOINTS)
        self.line_pub = rospy.Publisher("/line", Marker)
        self.pub = rospy.Publisher('/vel_cmd', Twist, queue_size=10)
        self.goalpub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.wallpts_pub = rospy.Publisher("/wallpts", Marker)
        self.prmstates.append(State(0.0, 0.0, 0.0))
        self.generate_PRM(NPOINTS, (self.w // 2, self.h // 2), (self.w / 2, self.h / 2))
        self.atgoal = True
        self.direction = ''
        self.pos = np.zeros(3)
        self.goal = np.zeros(3)
        self.object = False
        self.obj_slow = True
        self.is_localizing = True
        self.first_goal = True
        self.obstacles = []
        self.wallptmap = np.zeros((300, 300, 2))
        w_array = []
        print("Reading file...")
        w_array = np.genfromtxt('/home/kpochana/robotws/src/me169/data/wallptmap.csv', delimiter=',')
        i = 0
        for v in range(300):
            for u in range(300):
                self.wallptmap[v, u][0] = w_array[i]
                self.wallptmap[v, u][1] = w_array[i + 1]
                i += 2
        print('Made array with dimensions', np.shape(self.wallptmap), '...')
        print('Done :)')

    # CALLBACKS AND HELPERS ----------------------------------------------------

    def update_map(self, point):
        x = point[0]
        y = point[1]
        u = int((x - GRID_X) / RES)
        v = int((y - GRID_Y) / RES)
        self.wallptmap[v, u] = (u, v)
        u_range = np.arange(u - int(DIST_BLUR / RES), u + int(DIST_BLUR / RES), 1)
        v_range = np.arange(v - int(DIST_BLUR / RES), v + int(DIST_BLUR / RES), 1)
        for u_curr in u_range:
            for v_curr in v_range:
                nearest_pt = self.wallptmap[v_curr, u_curr]
                dist = np.linalg.norm(np.array([u_curr - nearest_pt[0], v_curr - nearest_pt[1]]))
                new_dist = np.linalg.norm(np.array([u_curr - u, v_curr - v]))
                if new_dist < dist:
                    self.wallptmap[v_curr, u_curr] = (u, v)
    
    def cb_obstacle(self, msg):
        points = msg.points
        added = 0
        obs = self.obstacles_added
        self.obstacles_added = 999
        for pt in points:
            point = (pt.x, pt.y)
            if not (point in self.obstacles):
                if point not in self.obstacle_dic:
                    self.obstacle_dic[point] = 1
                else:
                    self.obstacle_dic[point] += 1
                if self.obstacle_dic[point] >= OBSTACLE_THRESH:
                    self.obstacles = self.obstacles + [point]
                    added += 1
                    self.update_map(point)
        self.obstacles_added = obs
        self.obstacles_added += added
        self.obstacles_added /= 2.0
        # print(self.obstacles_added)
        
    def cb_localizing(self, msg):
        self.is_localizing = msg.data

    def cb_laser(self, msg):
        self.object = False
        self.obj_slow = True
        replan = False
        if self.obstacles_added > 1 and self.valid_path:
            # print('Checking path...', rospy.get_time())
            self.valid_path, self.loc = verifyPath([Node(State(self.pos[0], self.pos[1], self.pos[2]), self.wallptmap)] \
                                              + [Node(State(self.goal[0], self.goal[1], self.goal[2]), self.wallptmap)] \
                                              + self.path,
                                              self.wallptmap)
        if not self.valid_path:
            # print('Bad path detected...', rospy.get_time())
            self.object = True
            self.hard_stop()
            if not self.planning and self.obstacles_added < 1:
                self.planning = True
                print('Trying to replan to goal')
                self.generate_PRM(15, self.loc, (20, 20))
                new_msg = PoseStamped()
                new_msg.pose.position = self.goal_final.position
                new_msg.pose.orientation = self.goal_final.orientation
                new_msg.header.frame_id = 'map'
                replan = True
                self.goalpub.publish(new_msg)
            return
        ranges = msg.ranges
        angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)
        # print('min',msg.angle_min)
        # print('max',msg.angle_max)
        theta_to_goal = math.atan2(self.goal[1] - self.pos[1], 
                                     self.goal[0] - self.pos[0])
        pos_angle = math.atan2(np.sin(self.pos[2]), np.cos(self.pos[2]))
        # print("Facing goal?:", math.isclose(theta_to_goal - self.pos[2], 0.0, abs_tol=0.1))
        # print(theta_to_goal, "  ", pos_angle)
        xstat = []
        for i, dist in enumerate(ranges):
            curr_angle = angles[i]
            dist_key = round(dist, 2)

            # if (curr_angle, dist_key) not in self.laser_dic:
            #             self.laser_dic[(curr_angle, dist_key)] = 1
            # else:
            #     self.laser_dic[(curr_angle, dist_key)] += 1

            # if self.laser_dic[(curr_angle, dist_key)] <= LASER_THRESH:
            #     continue

            # if (curr_angle > -ANGLE_RANGE and curr_angle < ANGLE_RANGE):
                # if (dist <= RANGE_CUTOFF and dist >= RANGE_MIN): 
                    # if self.goal_final and \
                    #    not replan and \
                    #    not self.planning and \
                    #    math.isclose(theta_to_goal - self.pos[2], 0.0, abs_tol=0.1):
                    #     print('Trying to replan to goal')
                    #     new_msg = PoseStamped()
                    #     new_msg.pose.position = self.goal_final.position
                    #     new_msg.pose.orientation = self.goal_final.orientation
                    #     new_msg.header.frame_id = 'map'
                    #     replan = True
                    #     self.goalpub.publish(new_msg)

                    # if (curr_angle > -ANGLE_STOP and curr_angle < ANGLE_STOP and dist <= 0.3):
                        # self.object = True
            if dist > RANGE_MIN and dist < RANGE_MAX:
                x = dist * np.sin(curr_angle)
                y = dist * np.cos(curr_angle)
                # print("x: ", x)
                # print("y: ", y)
                if -0.5 * BOT_WIDTH < x and x < 0.5 * BOT_WIDTH \
                    and RANGE_MIN < y and y < D_OBJECT:
                    # xstat.append(x)
                    # print('OBJECT???', rospy.get_time())
                    self.object = True
                    self.hard_stop()
                    # print("Planning?:", self.planning)
                    # print("Replanning?:", replan) 
                    # print(not self.planning, not replan, math.isclose(theta_to_goal - pos_angle, 0.0, abs_tol=0.1))
                    if not self.planning and not replan and math.isclose(theta_to_goal - pos_angle, 0.0, abs_tol=0.1) and not self.first_goal:
                        self.planning = True 
                        print('Sneaky object: trying to replan to goal')
                        print('pos',self.pos)
                        print('goal',self.goal)
                        print('theta to goal', theta_to_goal)
                        print('theta pos', pos_angle)
                        self.generate_PRM(30, (int((self.pos[0] - GRID_X) / RES), int((self.pos[1] - GRID_Y) / RES)), (50, 50))
                        new_msg = PoseStamped()
                        new_msg.pose.position = self.goal_final.position
                        new_msg.pose.orientation = self.goal_final.orientation
                        new_msg.header.frame_id = 'map'
                        replan = True
                        self.goalpub.publish(new_msg)
                        # MOVE INTO SPECIAL MODE?????
                    break
        if not self.object:
            self.obj_slow = False
        # if len(xstat) > 0:
        #     print('min_x', np.min(xstat))
        #     print('max_x', np.max(xstat))
    
        # if not self.object:
        #     print('Doi i see no object :)', rospy.get_time())

    def plan(self, p, q):
        self.planning = True
        self.prmstates.append(State(p.x, p.y, self.get_theta(q)))
        self.nodes = []
        # self.publish_wallpts()
        self.nodes.append(Node(State(self.pos[0], self.pos[1], self.get_theta(q)), self.wallptmap))
        for state in self.prmstates:
            self.nodes.append(Node(state, self.wallptmap))
        
        print("Graph contains", len(self.nodes), "nodes")
        
        self.path = AStar(self.nodes, self.nodes[0], self.nodes[-1])
        print('PATH LENGTH = ', len(self.path))
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
            self.linemsg.points.append(Point(node.state.x, node.state.y, 0.0))

        self.goal = [self.path[0].state.x, self.path[0].state.y, self.path[0].state.theta]
        self.first_goal = True
        self.line_pub.publish(self.linemsg)
        self.planning = False
        self.valid_path = True

    def cb_goal(self, msg):
        assert(msg.header.frame_id == 'map')
        p = msg.pose.position
        q = msg.pose.orientation
        self.goal_final = msg.pose
        # self.goal = np.array([p.x, p.y, self.get_theta(q)])
        self.direction = ''
        self.atgoal = False
        print("Goal Received")
        self.plan(p, q)

    def cb_pose(self, msg):
        if not self.planning:
            assert(msg.header.frame_id == 'map')
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
                    self.first_goal = False
                    self.wheel_control = 0
                elif len(self.path) == 1:
                    #doi
                    self.goal = np.array([self.path[0].state.x, 
                                        self.path[0].state.y, 
                                        self.path[0].state.theta])
                    self.first_goal = False
                    self.wheel_control = 0
                    self.path = self.path[1:]
                    print("A* Complete: Heading to final goal")

            self.pos[0:2] = np.array([p.x, p.y])
            self.pos[2] = self.get_theta(q) 

            dir = self.goal[0:2] - self.pos[0:2] 
            dir_theta = np.arctan2(dir[1], dir[0])

            dist = np.linalg.norm(dir)
            v_x = 0.0
            w_z = 0.0
            if (not np.array_equal(self.goal, self.pos) and \
                self.wheel_control == 0):
                angle = self.choose_dir(dir_theta, self.pos[2])
                w_z = -LAMBDA * angle
                if math.isclose(angle, 0.0, abs_tol=0.1):
                    self.wheel_control = 1
            if (not np.array_equal(self.goal, self.pos) and \
                  self.wheel_control == 1):
                angle = self.choose_dir(dir_theta, self.pos[2])
                w_z = -LAMBDA_DRIVING * angle
                v_x = min(VEL_MAX, GAMMA * dist) * np.cos(angle)
                if dist < 0.05:
                    self.wheel_control = 2
            if (self.wheel_control == 2):
                v_x = 0.0
                angle = self.choose_dir(self.goal[2], self.pos[2])

                w_z = -LAMBDA * angle

                if (np.abs(angle) < 0.1):
                    if not np.array_equal(self.goal, self.pos):
                        print("REACHED WAYPOINT!")
                    self.goal = self.pos
                    self.wheel_control = 3

            if (self.object or self.planning):
                # print("Stopping!!")
                if v_x > 0:
                    v_x = 0.0
            
            if (abs(w_z) > OMEGA_MAX):
                if w_z < 0:
                    w_z = -1.0 * OMEGA_MAX
                else:
                    w_z = OMEGA_MAX
            
            if (math.isclose(w_z, 0.0, abs_tol=0.05)):
                w_z = 0.0
            elif (abs(w_z) < OMEGA_MIN):
                if w_z < 0:
                    w_z = -1.0 * OMEGA_MIN
                else:
                    w_z = OMEGA_MIN

            if self.planning:
                w_z = 0.0

            if self.obj_slow:
                v_x *= 0.75

            msg = Twist()
            msg.linear.x = v_x
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = w_z
            self.vx = v_x
            self.wz = w_z
            self.pub.publish(msg)
            # if self.generated:
            #     self.waypoints_pub.publish(self.markermsg)
        else:
            self.hard_stop()

    # HELPER FUNCTIONS ---------------------------------------------------------

    def hard_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.pub.publish(msg)

    def minmax(minimum, maximum, omegaz):
        assert(maximum >= minimum)
        return min(maximum, max(minimum, omegaz))

    def get_theta(self, q):
        x = 2.0 * np.arctan2(q.z, q.w)
        return x

    def get_map(self):
        # Wait 30sec for a map.
        rospy.loginfo("Waiting for a map...")
        self.h = 300
        self.w = 300
        self.mapmsg = rospy.wait_for_message('/map', OccupancyGrid, 30.0)
        self.map = np.array(self.mapmsg.data).reshape(self.h, self.w)
        self.resolution = self.mapmsg.info.resolution

    def generate_PRM(self, npoints, center, range):
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
        if not self.markermsg.points:
            self.markermsg.points = []
        while count < npoints:
            u = randint(-range[0], range[0] - 1) + center[0]
            v = randint(-range[1], range[1] - 1) + center[1]
            count += 1
            if self.map[v, u] == 0:
                self.points.append([u, v])
                x = float(u + 0.5) * self.resolution + GRID_X
                y = float(v + 0.5) * self.resolution + GRID_Y
                self.markermsg.points.append(Point(x, y, 0.0))
                self.prmstates.append(State(x, y, 0.0)) # x y theta
        self.generated = True

    def publish_wallpts(self):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.ns = "waypoints"
        msg.id = 0
        msg.type = Marker.POINTS
        msg.action = Marker.ADD
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        msg.scale.x = 0.03
        msg.scale.y = 0.03
        msg.scale.z = 0.03
        msg.color.a = 1.0
        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 1.0
        msg.type = 7
        points = []
        for v in range(len(self.wallptmap)):
            for u in range(len(self.wallptmap[v,:])):
                points.append(Point(float(self.wallptmap[v,u][0] + 0.5) * RES + GRID_X,
                                    float(self.wallptmap[v,u][1] + 0.5) * RES + GRID_Y,
                                    0.0))
        msg.points = points
        self.wallpts_pub.publish(msg)
    
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

    def grab_theta(self, msg):
        self.pos[2] = msg.position[3]

    


if __name__ == "__main__":
    
    rospy.init_node('planner')

    planner = SimplePlanner()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Planner spinning...")
    rospy.spin()
    rospy.loginfo("Planner stopped.")

