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
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist, Pose, TransformStamped, PoseWithCovarianceStamped


WALLTHRESHOLD = 0.1
FRACTIONAL_SCALE = 1.0 / 1000.0
GRID_X = -3.8100
GRID_Y = -3.8100
GRID_THETA = 0.0

class Localize: 

    def __init__(self):
        print("Initializing localization...")
        self.counter = 0
        self.localize_rate = 10
        self.initialized = False
        # 4.1
        self.pt_map_odom = PlanarTransform.unity()
        self.T_map_odom = self.pt_map_odom.toTransform()

        self.pos = [0.0, 0.0, 0.0]
        self.pt_odom_base = PlanarTransform.unity()
        self.T_odom_base = self.pt_odom_base.toTransform()
        self.pt_map_base = PlanarTransform.unity()
        self.T_map_base = self.pt_map_base.toTransform()

        # Set up publishers/subscribers
        rospy.Subscriber("/odom", Odometry, self.cb_odom)
        self.pose_pub = rospy.Publisher("/pose", PoseStamped, queue_size=10)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.cb_initial_pose)
        rospy.Subscriber("/scan", LaserScan, self.cb_laser)

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
        print("Setting up wallptmap")
        for v in range(self.h):
            print(str(round((v * self.w) / (self.h * self.w) * 100, 2)) + " done")
            for u in range(self.w):
                self.wallptmap[v, u] = self.calculate_point(u, v)

        # with open("/home/kpochana/robotws/src/me169/data/wallptmap.csv", 'w') as csvfile:
        #     writer = csv.writer(csvfile, delimiter=",",lineterminator="\n")
        #     for v in range(self.h):
        #         print("Writing " + str(round((v * self.w) / (self.h * self.w) * 100, 2)) + " done")
        #         writer.writerow(self.wallptmap[v,:])

        print("Set up wallptmap")
        self.d_theta = 0.0
        self.d_x = 0.0
        self.d_y = 0.0
        self.pt_map_laser = None
        # TODO: adjust this resolution!!!
        self.contlocal = True

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
            self.pt_map_laser = self.pt_map_odom * self.pt_odom_laser
            # print(self.pt_map_laser)
            
            if (self.contlocal and self.counter % self.localize_rate == 0):
                print("localizing...")
                r = np.zeros((2, 0))
                p = np.zeros((2, 0))
                ranges = msg.ranges
                angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

                for i, range in enumerate(ranges):
                    if range > msg.range_min and range < (msg.range_max / 1.0):
                        x = range * np.sin(angles[i])
                        y = range * np.cos(angles[i])
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
                        r = np.hstack([r, np.array([[r_x], [r_y]])])
                        u = int(round((r_x - GRID_X) / self.resolution))
                        v = int(round((r_y - GRID_Y) / self.resolution))
                        # try:
                        p_uv = self.nearestwallpt(u, v)
                        p_x = float(p_uv[0]) * self.resolution + GRID_X
                        p_y = float(p_uv[1]) * self.resolution + GRID_Y
                        p = np.hstack([p, np.array([[p_x], [p_y]])])

                self.minimize_least_squares(r, p)

                # # get the map to base frame from initial pose
                self.pt_map_base = self.pt_map_base * (FRACTIONAL_SCALE * PlanarTransform.basic(-self.d_x, -self.d_y, -self.d_theta))
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
        assert(len(r) == len(p))

        N = len(r)

        rx = r[0, :]
        ry = r[1, :]
        px = p[0, :]
        py = p[1, :]

        Rx = 1.0 / N * np.sum(rx)
        Ry = 1.0 / N * np.sum(ry)

        Px = 1.0 / N * np.sum(px)
        Py = 1.0 / N * np.sum(py)

        RR = 1.0 / N * np.sum(rx**2 + ry**2)
        RP = 1.0 / N * np.sum(rx * py - ry * px)

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

    