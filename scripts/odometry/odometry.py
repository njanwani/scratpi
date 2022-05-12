#!/usr/bin/env python3
#
#   odometry.py
#
#   Odometry node.  This
#   (a) converts both a body velocity command to wheel velocity commands.
#   (b) estimates the body velocity and pose from the wheel motions
#       and the gyroscope.
#
#   Node:       /odometry
#   Publish:    /odom                   geometry_msgs/TransJointState
#               TF odom -> base         geometry_msgs/TransformStamped
#               /wheel_command          sensor_msgs/JointState
#   Subscribe:  /vel_cmd                geometry_msgs/Twist
#               /wheel_state            sensor_msgs/JointState
#
import math
import numpy as np
import rospy
import tf2_ros

from geometry_msgs.msg import Point, Quaternion, Twist
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg      import Odometry
from sensor_msgs.msg   import JointState


#
#   Constants
#
GEAR_RATIO  = 45
PPR = 16
DT = 0.01
MAX_DT = 0.25
K = 200
LMBDA = 0.9
T_CORR = 1.0
K_1 = 1
PWM_RATIO = 24

VEL_TO_PWM_SLOPE = 9.5
VEL_TO_PWM_OFFSET = 30

R = 0.0326  # Wheel radius
d = 0.067   # Halfwidth between wheels
J = np.array([[R / 2, - R / 2], 
              [- R / (2 * d), - R / (2 * d)]])
Jinv = np.linalg.inv(J)

#
#   Odometry Object
#
class OdometryObj:
    # Initialize.
    def __init__(self):
        # Set the initial pose to zero.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Create a publisher to send wheel commands.
        self.pub_wcmd = rospy.Publisher('/wheel_command', JointState,
                                        queue_size=3)

        # Create a publisher to send odometry information.
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)

        # Create a TF2 transform broadcaster.
        self.brd_tf = tf2_ros.TransformBroadcaster()

        # Create a subscriber to listen to twist commands.
        rospy.Subscriber('/vel_cmd', Twist, self.cb_vel_cmd)

        # Create a subscriber to listen to wheel state.
        rospy.Subscriber('/wheel_state', JointState, self.cb_wheel_state)


    # Velocity Command Message Callback
    def cb_vel_cmd(self, msg):
        global cmdvel, cmdspin

        # Grab the forward and spin (velocity) commands.
        cmdvel  = msg.linear.x
        cmdspin = msg.angular.z
        
        # CONVERT THE BODY VELOCITY COMMANDS TO L/R WHEEL COMMANDS
        kin = Jinv @ np.array([[cmdvel], [cmdspin]])

        l_psidot = kin[0, 0]
        r_psidot = kin[1, 0]

        # Create the wheel command msg and publish.  Note the incoming
        # message does not have a time stamp, so generate one here.
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name         = ['leftwheel', 'rightwheel']
        msg.velocity     = [l_psidot,    -r_psidot    ]
        # ADD IN GYRO?

        self.pub_wcmd.publish(msg)


    # Wheel State Message Callback
    def cb_wheel_state(self, msg):
        # Grab the timestamp, wheel and gyro position/velocities.
        global cmdvel, cmdspin
        print(msg)
        timestamp = msg.header.stamp
        lpsi = msg.velocity[msg.name.index('leftwheel')]
        rpsi = -msg.velocity[msg.name.index('rightwheel')]
        # ADD IN GYRO??


        dp = R / 2 * (lpsi * DT - rpsi * DT)
        # dtheta = R / (2 * d) * (-lpsi * DT - rpsi * DT)
        dtheta = msg.velocity[msg.name.index('theta_IMU')]
        # Update the pose.
        self.x      += dp * np.cos(self.theta + dtheta / 2)
        self.y      += dp * np.sin(self.theta + dtheta / 2)
        # self.theta  += dtheta
        self.theta = msg.position[msg.name.index('theta_IMU')]

        # Convert to a ROS Point, Quaternion, Twist (lin&ang veloocity).
        p = Point(self.x, self.y, 0.0)
        q = Quaternion(0.0, 0.0, math.sin(self.theta/2), math.cos(self.theta/2))
        t = Twist(Vector3(cmdvel, 0.0, 0.0), Vector3(0.0, 0.0, cmdspin))
        print("Lpsi: ", f'{lpsi:.3f}', "Rpsi: ", f'{rpsi:.3f}')
        print(p)

        # Create the odometry msg and publish (reuse the time stamp).
        msg = Odometry()
        msg.header.stamp            = timestamp
        msg.header.frame_id         = 'odom'
        msg.child_frame_id          = 'base'
        msg.pose.pose.position      = p
        msg.pose.pose.orientation   = q
        msg.twist.twist             = t
        self.pub_odom.publish(msg)

        # Create the transform msg and broadcast (reuse the time stamp).
        msg = TransformStamped()
        msg.header.stamp            = timestamp
        msg.header.frame_id         = 'odom'
        msg.child_frame_id          = 'base'
        msg.transform.translation   = p
        msg.transform.rotation      = q
        self.brd_tf.sendTransform(msg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    global cmdvel, cmdspin
    
    cmdvel = 0
    cmdspin = 0

    rospy.init_node('odometry')

    # Instantiate the Odometry object
    odometry = OdometryObj()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Odometry spinning...")
    rospy.spin()
    rospy.loginfo("Odometry stopped.")