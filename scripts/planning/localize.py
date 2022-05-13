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
from geometry_msgs.msg import PoseArray, PoseStamped, Point, Quaternion, Twist, Pose, TransformStamped, PoseWithCovarianceStamped


WALLTHRESHOLD = 0.1
FRACTIONAL_SCALE = 1.0 / 10.0
FRACTIONAL_SCALE_THETA = 2.0 / 10.0
MAX_CART_SCALE = 0.03
MAX_THETA_SCALE = np.pi / 20.0
THROTTLE = 0.00075
GRID_X = -3.8100
GRID_Y = -3.8100
GRID_THETA = 0.0


class Localize: 

    def __init__(self):
        print("Initializing localization...")
        self.counter = 0
        self.localize_rate = 25
        self.initialized = False
        # 4.1
        self.pt_map_odom = PlanarTransform.unity()
        self.T_map_odom = self.pt_map_odom.toTransform()

        self.pos = [0.0, 0.0, 0.0]
        self.pt_odom_base = PlanarTransform.unity()
        self.T_odom_base = self.pt_odom_base.toTransform()
        self.pt_map_base = PlanarTransform.unity()
        self.T_map_base = self.pt_map_base.toTransform()

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
        # print("Dims of wallpts: ", self.wallpts.shape)
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

        print("Reading file...")
        w_array = np.genfromtxt('/home/kpochana/robotws/src/me169/data/wallptmap.csv', delimiter=',')

        # Set up publishers/subscribers
        rospy.Subscriber("/odom", Odometry, self.cb_odom)
        self.pose_pub = rospy.Publisher("/pose", PoseStamped, queue_size=10)
        self.pose_desired_pub = rospy.Publisher("/pose_desired", PoseStamped, queue_size=10)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.cb_initial_pose)
        rospy.Subscriber("/scan", LaserScan, self.cb_laser)
        self.array_pub = rospy.Publisher("/posearray", PoseArray)


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
        # self.contlocal = True

        print("Done initializing localization.")

    def get_map(self):
        # Wait 30sec for a map.
        rospy.loginfo("Waiting for a map...")
        self.mapmsg = rospy.wait_for_message('/map', OccupancyGrid, 30.0)
        self.map = np.array(self.mapmsg.data).reshape(self.h, self.w)
        self.resolution = self.mapmsg.info.resolution

    def cb_initial_pose(self, msg):
        print("Got initial pose")
        # 4.4
        timestamp = msg.header.stamp
        pose = msg.pose.pose

        # get the map to base frame from initial pose
        self.pt_map_base = PlanarTransform.fromPose(pose)
        self.T_map_base = self.pt_map_base.toTransform()

        # back-calculate? the transform for the odom in map frame (or is it reversed...)
        self.pt_map_odom = self.pt_map_base * self.pt_odom_base.inv()
        self.T_map_odom = self.pt_map_odom.toTransform()

    def cb_odom(self, msg):
        # print("Got odometry.")
        # 4.3
        # use the old time-stamp
        timestamp = msg.header.stamp
        header = msg.header

        # multiply transforms to get into base in reference of the map frame
        self.pt_odom_base = PlanarTransform.fromPose(msg.pose.pose)
        self.T_odom_base = self.pt_odom_base.toTransform()

        # multiply transforms
        self.pt_map_base = self.pt_map_odom * self.pt_odom_base
        self.T_map_base = self.pt_map_base.toTransform()
        
        # Publish the transformed position
        msg =                   PoseStamped()
        # msg.header =            header
        msg.header.stamp =      timestamp
        msg.header.frame_id =   "map"
        msg.pose =              self.pt_map_base.toPose()

        self.pose_pub.publish(msg)
        # print("Published to pose.")


    def cb_laser(self, msg, init=False):
        # print("Got laser")
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
            # print(self.pt_odom_laser.x(), self.pt_odom_laser.y())
            self.pt_map_laser = self.pt_map_odom * self.pt_odom_laser #* PlanarTransform.basic(0.0, 0.0, -np.pi / 2.0)
            # print(self.pt_map_laser)
            counter = 0
            if (self.contlocal and self.counter % self.localize_rate == 0):
                # print("localizing...")
                r = np.zeros((2, 0))
                p = np.zeros((2, 0))
                ranges = msg.ranges
                angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)[::5]

                for i, range in enumerate(ranges[::5]):
                    if range > msg.range_min and range < (msg.range_max / 2.0):
                        y = range * np.sin(angles[i])
                        x = range * np.cos(angles[i])
                        # print("x: ", x)
                        # print("y: ", y)
                        pt_r = self.pt_map_laser * PlanarTransform(x, y, 0.0, 1.0)
                        # print("r_x: ", pt_r.x())
                        # print("r_y: ", pt_r.y())
                        # r_x_g, r_y_g = self.map_to_grid(copy(pt_r.x()), copy(pt_r.y()))
                        r_x, r_y = pt_r.x(), pt_r.y()
                        # r_y = self.map_to_grid(pt_r.y())
                        # print("r_x_g: ", r_x)
                        # print("r_y_g: ", r_y)
                        u = int((r_x - GRID_X) / self.resolution)
                        v = int((r_y - GRID_Y) / self.resolution)
                        try:
                            p_uv = self.nearestwallpt(u, v)
                            p_x = float(p_uv[0] + 0.5) * self.resolution + GRID_X
                            p_y = float(p_uv[1] + 0.5) * self.resolution + GRID_Y
                            p = np.hstack([p, np.array([[p_x], [p_y]])])
                            r = np.hstack([r, np.array([[r_x], [r_y]])])
                            counter += 1
                            # if counter > 3:
                            #     break
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
                self.minimize_least_squares(r, p)

                # # get the map to base frame from initial pose
                dx = self.bound(MAX_CART_SCALE, self.d_x * FRACTIONAL_SCALE)
                dy = self.bound(MAX_CART_SCALE, self.d_y * FRACTIONAL_SCALE)
                dtheta = self.bound(MAX_THETA_SCALE, self.d_theta * FRACTIONAL_SCALE_THETA)

                if (np.linalg.norm(np.array([dx, dy, dtheta])) < THROTTLE):
                    dx = 0.0
                    dy = 0.0
                    dtheta = 0.0

                print(dx, dy, dtheta)
                # print("BEFORE:", self.pt_map_base)
                # Publish the transformed position
                desired_dir_msg =                   PoseStamped()
                # msg.header =            header
                desired_dir_msg.header.stamp =      rospy.Time.now()
                desired_dir_msg.header.frame_id =   "map"
                desired_dir_msg.pose.position =      Point(self.pt_map_base.x(),
                                                           self.pt_map_base.y(),
                                                           0.0)
                
                desired_theta = np.arctan2(dy, dx)
                desired_dir_msg.pose.orientation =   Quaternion(0.0,
                                                                0.0,
                                                                np.sin(desired_theta / 2.0), 
                                                                np.cos(desired_theta / 2.0))
                self.pose_desired_pub.publish(desired_dir_msg)

                self.pt_map_base = PlanarTransform.basic(dx, dy, dtheta) * self.pt_map_base #* PlanarTransform.basic(0.0, 0.0, dtheta)
                # print("AFTER:", self.pt_map_base, "\n---")

                self.T_map_base = self.pt_map_base.toTransform()

                # back-calculate? the transform for the odom in map frame (or is it reversed...)
                self.pt_map_odom = self.pt_map_base * self.pt_odom_base.inv()
                self.T_map_odom = self.pt_map_odom.toTransform()

            t = TransformStamped()

            t.header.stamp = tfmsg.header.stamp
            t.header.frame_id = 'map'
            t.transform = self.T_map_odom
            t.child_frame_id = 'odom'

            self.tfbroadcast.sendTransform(t)
            # print("Sending transform to tf2.")

    def get_theta(self, q):
        x = 2.0 * np.arctan2(q.z, q.w)
        return x

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
        # assert(len(r) == len(p))

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

    