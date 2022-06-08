#!/usr/bin/env python3
#
#   localize.py
#
#
import math
import sys
import time
import rospy
import smbus
import numpy as np
import tf_conversions
import tf2_ros
import csv
from copy import copy
from operator import truediv

sys.path.insert(0, '/home/kpochana/robotws/src/cs169-shared/me169_shared/scripts/')
from PlanarTransform import *

from sensor_msgs.msg import JointState, LaserScan
from nav_msgs.msg      import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, PoseStamped, Point, Quaternion, Twist, Pose, TransformStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool


WALLTHRESHOLD = 0.1
FRACTIONAL_SCALE = 1.0 / 10.0
FRACTIONAL_SCALE_MOVING = 1.0 / 20.0
FRACTIONAL_SCALE_TURNING = 1.0 / 100.0
MAX_CART_SCALE = 0.03
MAX_THETA_SCALE = np.pi / 20.0
THROTTLE = 0.05
OBSTACLE_CUTOFF = 1.0
MAX_DIST = 0.2
GRID_X = -3.8100
GRID_Y = -3.8100
GRID_THETA = 0.0
OBSTACLE_THRESH = 10


class Localize: 

    def __init__(self):
        print("Initializing localization...")
        self.counter = 0
        self.localize_rate = 10
        self.initialized = False
        self.obstacle_dic = {}
        # 4.1
        self.pt_map_odom = PlanarTransform.unity()

        self.pos = [0.0, 0.0, 0.0]
        self.pt_odom_base = PlanarTransform.unity()

        # 4.2
        self.mapmsg = None
        self.map = None
        self.h = 300
        self.w = 300
        self.resolution = 300.0
        self.get_map()
        print("RESOLUTION: ", self.resolution)


        # 4.5
        self.tfbroadcast = None
        self.tfBuffer = None
        self.tfListen = None
        self.cb_laser(None, True)

        # 6
        self.contlocal = False
        self.wallpts = np.zeros((0, 2), dtype=np.int)
        self.find_neighbors()
        print("Found Neighbors")
        self.wallptmap = np.zeros((self.h, self.w, 2))
        # print("Setting up wallptmap")
        # for v in range(self.h):
        #     print(str(round((v * self.w) / (self.h * self.w) * 100, 2)) + " done")
        #     for u in range(self.w):
        #         self.wallptmap[v, u] = self.calculate_point(u, v)

         
        w_array = []
        # for v in range(self.h):
        #     for u in range(self.w):
        #         w_array.append(self.wallptmap[v, u][0])
        #         w_array.append(self.wallptmap[v, u][1])
        # print('Writing file using array with dimensions', np.shape(w_array), '...')
        # np.savetxt('/home/kpochana/robotws/src/me169/data/wallptmap.csv', w_array, delimiter=',')
        # print('Done! :)')

        self.obstacles = []

        print("Reading file...")
        w_array = np.genfromtxt('/home/kpochana/robotws/src/me169/data/wallptmap.csv', delimiter=',')

        # Set up publishers/subscribers
        rospy.Subscriber("/odom", Odometry, self.cb_odom)
        self.pose_pub = rospy.Publisher("/pose", PoseStamped, queue_size=10)
        self.pose_desired_pub = rospy.Publisher("/pose_desired", PoseStamped, queue_size=10)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.cb_initial_pose)
        rospy.Subscriber("/scan", LaserScan, self.cb_laser, queue_size=1)
        rospy.Subscriber("/vel_cmd", Twist, self.cb_vel, queue_size=10)
        self.array_pub = rospy.Publisher("/posearray", PoseArray)
        self.obstacles_pub = rospy.Publisher("/obstacles", Marker)
        self.localizing = rospy.Publisher("/localizing", Bool)


        i = 0
        for v in range(self.h):
            for u in range(self.w):
                self.wallptmap[v, u][0] = w_array[i]
                self.wallptmap[v, u][1] = w_array[i + 1]
                i += 2

        print('Made array with dimensions', np.shape(self.wallptmap), '...')
        print('Done :)')

        # newgrid = np.zeros_like(self.wallptmap)
        # newgrid = self.wallptmap[:, :, 0]**2 + self.wallptmap[:, :, 1]**2
        # import matplotlib.pyplot as plt
        # plt.figure()
        # plt.imshow(newgrid)
        # plt.show()

        # with open("/home/kpochana/robotws/src/me169/data/wallptmap.csv", 'w') as csvfile:
        #     writer = csv.writer(csvfile, delimiter=",",lineterminator="\n")
        #     for v in range(self.h):
        #         print("Writing " + str(round((v * self.w) / (self.h * self.w) * 100, 2)) + " done")
        #         writer.writerow(self.wallptmap[v,:])

        print("Set up wallptmap!")
        self.d_theta = 0.0
        self.d_x = 0.0
        self.d_y = 0.0
        self.pt_map_laser = None

        # Values of whether bot is moving
        self.v_x = 0.0
        self.w_z = 0.0
        self.is_localizing = True

        print("Done initializing localization.")

    def get_map(self):
        # Wait 30sec for a map.
        rospy.loginfo("Waiting for a map...")
        self.mapmsg = rospy.wait_for_message('/map', OccupancyGrid, 30.0)
        self.map = np.array(self.mapmsg.data).reshape(self.h, self.w)
        self.resolution = self.mapmsg.info.resolution


    def cb_vel(self, msg):
        # Basically checking if bot is moving

        self.v_x = msg.linear.x
        self.w_z = msg.angular.z

        

    def cb_initial_pose(self, msg):
        print("Got initial pose")
        # 4.4
        timestamp = msg.header.stamp
        pose = msg.pose.pose

        # back-calculate? the transform for the odom in map frame (or is it reversed...)
        self.pt_map_odom = PlanarTransform.fromPose(pose) * self.pt_odom_base.inv()
        self.contlocal = True

    def cb_odom(self, msg):
        # print("Got odometry.")
        # 4.3
        # use the old time-stamp
        timestamp = msg.header.stamp
        header = msg.header

        # multiply transforms to get into base in reference of the map frame
        self.pt_odom_base = PlanarTransform.fromPose(msg.pose.pose)

        # multiply transforms
        pt_map_base = self.pt_map_odom * self.pt_odom_base
        
        # Publish the transformed position
        msg =                   PoseStamped()
        msg.header.stamp =      timestamp
        msg.header.frame_id =   "map"
        msg.pose =              pt_map_base.toPose()

        self.pose_pub.publish(msg)
        # print("Published to pose.")


    def cb_laser(self, msg, init=False):
        if (init):
            # print("Intializing map(laser) frame.")
            # First create a TF2 listener.  This implicily fills a local
            # buffer, so we can always retrive the transforms we want.
            self.tfBuffer = tf2_ros.Buffer()
            self.tflisten = tf2_ros.TransformListener(self.tfBuffer)
            # Then create a TF2 transform broadcaster.
            self.tfbroadcast = tf2_ros.TransformBroadcaster()
            # Give the broadcaster time to connect, then send the initial transform.
            rospy.sleep(0.25)
            tfmsg = TransformStamped()
            tfmsg.header.stamp    = rospy.Time.now()
            tfmsg.header.frame_id = 'map'
            tfmsg.child_frame_id  = 'odom'
            tfmsg.transform       = PlanarTransform.unity().toTransform()
            self.tfbroadcast.sendTransform(tfmsg)
            self.initialized = True
        elif self.initialized:
            # Assume we received a scan message (scanmsg).  Use TF to
            # look up the matching transform.  Give it up to 100ms, in
            # case Linux has (temporarily) swapped out the odometry node.
            try:
                tfmsg = self.tfBuffer.lookup_transform('odom',
                                                        msg.header.frame_id,
                                                        msg.header.stamp,
                                                        rospy.Duration(0.1))
            except:
                return
            self.counter += 1
            self.pt_odom_laser = PlanarTransform.fromTransform(tfmsg.transform)
            self.pt_map_laser = self.pt_map_odom * self.pt_odom_laser

            if (self.contlocal):
                # print("localizing...")
                self.is_localizing = False

                r = np.zeros((2, 0))
                p = np.zeros((2, 0))
                ranges = msg.ranges
                angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)
                new_obstacles = []
                pts_ctr = 0
                for i, range in enumerate(ranges):
                    if range > (msg.range_min * 2.0) and range < (msg.range_max):
                        pts_ctr += 1
                        y = range * np.sin(angles[i])
                        x = range * np.cos(angles[i])
                        pt_r = self.pt_map_laser * PlanarTransform(x, y, 0.0, 1.0)
                        r_x, r_y = pt_r.x(), pt_r.y()
                        u = int((r_x - GRID_X) / self.resolution)
                        v = int((r_y - GRID_Y) / self.resolution)
                        # print('here')
                        try:
                            p_uv = self.nearestwallpt(u, v)
                            p_x = (float(p_uv[0])+0.5) * self.resolution + GRID_X
                            p_y = (float(p_uv[1])+0.5) * self.resolution + GRID_Y
                            distance = self.dist(r_x, r_y, p_x, p_y)
                            # print('dist =',distance)
                            point = (round(r_x, 2), round(r_y, 2))
                            if (distance < MAX_DIST):
                                p = np.hstack([p, np.array([[p_x], [p_y]])])
                                r = np.hstack([r, np.array([[r_x], [r_y]])])
                            elif (range < OBSTACLE_CUTOFF and not (point in self.obstacles) and not (point in new_obstacles)):
                                if point not in self.obstacle_dic:
                                    self.obstacle_dic[point] = 1
                                else:
                                    self.obstacle_dic[point] += 1
                                
                                if self.obstacle_dic[point] > OBSTACLE_THRESH:
                                    new_obstacles = new_obstacles + [point]
                        except:
                            continue
                
                poses = []
                density = 5
                for i, wallpt in enumerate(np.swapaxes(p, 0, 1)):
                    if i % density != 0:
                        continue
                    pose = Pose()
                    pose.position = Point(r[0][i], r[1][i], 0.0)
                    try:
                        theta = np.arctan2((wallpt[1] - r[1][i]), (wallpt[0] - r[0][i]))
                    except:
                        print("Catch divide by zero")
                        continue
                    pose.orientation = Quaternion(0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))
                    poses.append(pose)

                poses_msg = PoseArray()
                poses_msg.header.stamp = rospy.Time.now()
                poses_msg.header.frame_id = 'map'
                poses_msg.poses = poses

                self.array_pub.publish(poses_msg)
                consideredpts = len(r[0, :])
                # Logic for how to handle the scale of localization
                if consideredpts > 50:
                    self.minimize_least_squares(r, p)
                    if (math.isclose(self.v_x, 0.0) and math.isclose(self.w_z, 0.0)):
                        delta = FRACTIONAL_SCALE * (consideredpts / pts_ctr) * PlanarTransform.basic(self.d_x, self.d_y, self.d_theta)
                    else:
                        delta = FRACTIONAL_SCALE_MOVING * (consideredpts / pts_ctr) * PlanarTransform.basic(self.d_x, self.d_y, self.d_theta)
                    self.pt_map_odom = delta * self.pt_map_odom
                    if  (np.linalg.norm(np.array([self.d_x, self.d_y, self.d_theta])) > THROTTLE):
                        self.is_localizing = True

                
                if (self.contlocal and not self.is_localizing):
                    # self.obstacles = new_obstacles
                    # print('Old', self.obstacles.shape)
                    # print('New', new_obstacles.shape)
                    self.obstacles = self.obstacles + new_obstacles
                    # print('here')
                    # print('Obstacles example: ', self.obstacles[0])
                    self.publish_obstacles()


            t = TransformStamped()

            t.header.stamp = tfmsg.header.stamp
            t.header.frame_id = 'map'
            t.transform = self.pt_map_odom.toTransform()
            t.child_frame_id = 'odom'

            self.tfbroadcast.sendTransform(t)

            b = Bool()
            b.data = self.is_localizing

            self.localizing.publish(b)

            # print("Sending transform to tf2.")

    def publish_obstacles(self):
        self.markermsg = Marker()
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
        self.markermsg.scale.x = 0.03
        self.markermsg.scale.y = 0.03
        self.markermsg.scale.z = 0.03
        self.markermsg.color.a = 1.0
        self.markermsg.color.r = 0.0
        self.markermsg.color.g = 0.0
        self.markermsg.color.b = 0.0
        self.markermsg.type = 7
        points = []
        for item in self.obstacles:
            points.append(Point(item[0], item[1], 0.0))
        self.markermsg.points = points

        self.obstacles_pub.publish(self.markermsg)

    def get_theta(self, q):
        x = 2.0 * np.arctan2(q.z, q.w)
        return x

    def dist(self, x1, y1, x2, y2):
        return ((x2 - x1)**2 + (y2 - y1)**2)**0.5

    def bound(self, magnitude, val):
        assert(magnitude >= 0)
        return val
        if val < 0:
            return max(-1.0 * magnitude, val)
        return min(magnitude, val)

    def map_to_grid(self, x, y):
        x_g = np.cos(GRID_THETA) * (x - GRID_X) + np.sin(GRID_THETA) * (y - GRID_Y)
        y_g = -1.0 * np.sin(GRID_THETA) * (x - GRID_X) + np.cos(GRID_THETA) * (y - GRID_Y)
        return (round(x_g), round(y_g))

    def find_neighbors(self):
        for v in range(self.h):
            for u in range(self.w):
                if self.map[v, u] > WALLTHRESHOLD:
                    # Also check the adjacent pixels in a 3x3 grid.
                    adjacent = self.map[max(0,v-1):min(self.h, v+2), max(0, u-1):min(self.w, u+2)]
                    if not np.all(adjacent > WALLTHRESHOLD):
                        self.wallpts = np.vstack([self.wallpts, np.array([u, v])])

    def calculate_point(self, u, v):
        # print("Calculating point")
        return self.wallpts[np.argmin(np.sum((np.array([u, v]) - self.wallpts)**2, axis=1))]

    def nearestwallpt(self, u, v):
        return self.wallptmap[v, u]

    def minimize_least_squares(self, r: np.array, p: np.array):

        rx = r[0, :]
        ry = r[1, :]
        px = p[0, :]
        py = p[1, :]

        # rx = np.swapaxes(rx)
        # ry = np.swapaxes(ry)
        # px = np.swapaxes(px)
        # py = np.swapaxes(py)

        assert(len(rx) == len(px))

        N = float(len(rx))

        Rx = np.mean(rx)
        Ry = np.mean(ry)

        Px = np.mean(px)
        Py = np.mean(py)

        RR = np.mean(rx**2 + ry**2)
        RP = np.mean(rx * py - ry * px)

        self.d_theta = (RP - (Rx * Py - Ry * Px)) / (RR - (Rx**2 + Ry**2))
        self.d_x = Px - Rx + Ry * self.d_theta
        self.d_y = Py - Ry - Rx * self.d_theta


if __name__ == "__main__":
    
    rospy.init_node('localize')

    localizer = Localize()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Localizer spinning...")
    rospy.spin()
    rospy.loginfo("Localizer stopped.")

    