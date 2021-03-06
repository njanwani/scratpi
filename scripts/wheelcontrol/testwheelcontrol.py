#!/usr/bin/env python3
#
#   wheelcontrol_skeleton.py
#
#   This is a skelenton for the implementation of the Wheel-Control node.  Beyond
#   the basic, it should
#     - stops if no commands are received after 0.25sec
#     - filter the commands to avoid spikes (high torque/current loads)
#     - filters the actual velocity
#     - adds feedback to perfectly achieve the velocity commands
#
#   Node:       /wheelcontrol
#   Publish:    /wheel_state            sensor_msgs/JointState
#               /wheel_desired          sensor_msgs/JointState
#   Subscribe:  /wheel_command          sensor_msgs/JointState
#
#   Other Inputs:   Encoder Channels (GPIO)
#   Other Outputs:  Motor Driver Commands (via I2C)
#
import math
import sys
import time
import rospy
import smbus
import numpy as np

import encoder as motor_encoder
import driver as motor_driver
import gyro

from sensor_msgs.msg import JointState

GEAR_RATIO  = 45
PPR = 16
DT = 0.01
MAX_DT = 0.25
K = 200
LMBDA = 0.9
T_CORR = 1.0 # 0.3
K_1 = 1
PWM_RATIO = 24
T_CMD = 0.25
T_VEL = 1.0 # 0.2

VEL_TO_PWM_SLOPE = [12, 11]
VEL_TO_PWM_OFFSET = [40, 30]

WHEEL_RADIUS = 0.0326
WHEEL_TO_WHEEL = 0.067
J = np.array([[WHEEL_RADIUS / 2, - WHEEL_RADIUS / 2], 
            [- WHEEL_RADIUS / (2 * WHEEL_TO_WHEEL), - WHEEL_RADIUS / (2 * WHEEL_TO_WHEEL)]])

#
#   Command Callback Function
#
#   Save the command and the time received.
#
def callback_command(msg):
    # Check the message?
    global cmdvel, cmdtime

    # Note the current time (to timeout the command).
    now = rospy.Time.now()

    # Save...
    cmdvel  = msg.velocity # add generalizability later?
    cmdtime = now


def v_to_PWM(vel, side):
    lateral = 0 if side == 'l' else 1
    if vel >= 0:
        return VEL_TO_PWM_SLOPE[lateral] * vel + VEL_TO_PWM_OFFSET[lateral]
    else:
        return VEL_TO_PWM_SLOPE[lateral] * vel - VEL_TO_PWM_OFFSET[lateral]

#
#   Timer Callback Function
#
def callback_timer(event):
    # Note the current time to compute dt and populate the ROS messages.
    global  lastpos, puddes, pubact, encoder, driver, gyro, sub, cmdvel, cmdtime, \
            L_dist, R_dist, L_vel, R_vel, L_PWM, R_PWM, L_vel_des, R_vel_des, \
            omega_enc, omega_imu, theta_enc, theta_imu
    now = rospy.Time.now()
    # Process the encoders, convert to wheel angles!
    L_pos = encoder.leftencoder() / (GEAR_RATIO * PPR) * 2 * math.pi
    R_pos = encoder.rightencoder() / (GEAR_RATIO * PPR) * 2 * math.pi

    # calculate velocity...
    L_vel += DT / T_VEL * ((L_pos - lastpos[0]) / DT - L_vel)
    R_vel += DT / T_VEL * ((R_pos - lastpos[1]) / DT - R_vel)
    print("Left: ", f'{L_vel:.3f}', "Right: ", f'{R_vel:.3f}')
    L_PWM = 100
    R_PWM = 153
    lastpos = [L_pos, R_pos]

    try:
        # Send wheel commands.
        driver.left(L_PWM)
        driver.right(R_PWM)
        #print("left position = ", L_pos, "\tleft velocity = ", L_vel)
    except Exception as e:
        print(e)
        print('Could not send wheel PWM...')
        print('\tL_PWM =', L_PWM)
        print('\tR_PWM =', R_PWM)
        print('\tl_error =', l_error)
        print('\tr_error =', r_error)
        print('\tL_vel_des =', L_vel_des)
        print('\tR_vel_des =', R_vel_des)
        print('\tT_CORR =', T_CORR)

    # Publish the actual wheel state
    # msg = JointState()
    # msg.header.stamp = now
    # msg.name         = ['leftwheel', 'rightwheel', 'theta_encoder', 'theta_IMU']
    # msg.position     = [L_pos, R_pos, theta_enc, theta_imu]
    # msg.velocity     = [L_vel, R_vel, omega_enc, omega_imu]
    # msg.effort       = [0.0, 0.0, 0.0, 0.0]
    # pubact.publish(msg)
    # #print("published actual to ", msg)
    # #print(msg)

    # # Publish the desired wheel state
    # msg = JointState()
    # msg.header.stamp = now
    # msg.name         = ['leftwheel', 'rightwheel']
    # msg.position     = [L_dist, R_dist]
    # msg.velocity     = [L_vel_des, R_vel_des]
    # msg.effort       = [L_PWM, R_PWM]
    # pubdes.publish(msg)
    # print("published desired to ", msg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    global  lastpos, puddes, pubact, encoder, driver, gyro, sub, cmdvel, cmdtime, \
            L_dist, R_dist, L_vel, R_vel, L_PWM, R_PWM, L_vel_des, R_vel_des, \
            omega_enc, omega_imu, theta_enc, theta_imu
    L_dist = 0.0
    R_dist = 0.0
    L_vel = 0.0
    R_vel = 0.0
    L_PWM = 0.0
    R_PWM = 0.0
    L_vel_des = 0.0
    R_vel_des = 0.0
    omega_enc = 0.0
    omega_imu = 0.0
    theta_enc = 0.0
    theta_imu = 0.0

    cmdvel = [0.0, 0.0]
    cmdtime = rospy.Time(0)
    lastpos = [0.0, 0.0]

    i2cbus = smbus.SMBus(1)

    rospy.init_node('wheelcontrol')

    # Inititlize the low level.
    encoder = motor_encoder.Encoder()
    driver  = motor_driver.Driver(i2cbus)
    gyro = gyro.Gyro(i2cbus)

    # Create a publisher to send the wheel desired and actual (state).
    pubdes = rospy.Publisher('/wheel_desired', JointState, queue_size=10)
    pubact = rospy.Publisher('/wheel_state',   JointState, queue_size=10)

    # Create a subscriber to listen to wheel commands.
    sub = rospy.Subscriber("/wheel_command", JointState, callback_command)

    # Create the timer.
    duration = rospy.Duration(DT)
    dt       = duration.to_sec()
    timer    = rospy.Timer(duration, callback_timer)


    # Spin while the callbacks are doing all the work.
    rospy.loginfo("Running with dt = %.3f sec..." % dt)
    rospy.spin()
    rospy.loginfo("Stopping...")

    # Stop the timer (if not already done).
    timer.shutdown()

    # Clean up the low level.
    driver.shutdown()
    encoder.shutdown()
    gyro.shutdown()
