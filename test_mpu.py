from __future__ import division
import time
import math as m
import numpy as np
import array as arr 
import timeit
import threading
# Import the PCA9685 module.

import Adafruit_PCA9685
# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
# pwm = Adafruit_PCA9685.PCA9685()
mpu = Adafruit_PCA9685.MPU6050()

# Configure min and max servo pulse lengths
servo_min = 100  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

###
Roll = 0
Pitch = 0
# preTimeGet = 0
def getRollPitch_MPU():
    global Roll, Pitch # ,preTimeGet
    Roll, Pitch = mpu.get_RP_Angle()
    print("Goc truc X: %.1f" %Roll  ,  "Goc truc Y: %.1f" %Pitch)
    threading.Timer(0.5,getRollPitch_MPU).start()
    # timeGet = timeit.default_timer()
    # print(timeGet - preTimeGet, " (seconds)")
    # preTimeGet = timeGet
##
getRollPitch_MPU()
def PID(setpoint,current_value,kp,ki,kd):

    sample_time = 0.02
    err = setpoint - current_value # roll,pitch angle 
    err_p =0 
    iterm_p = 0

    #k term 
    kterm = err*kp

    #iterm
    iterm = iterm_p + ki*err*sample_time 

    #dterm 
    dterm = K_d*(err-err_p)/sample_time

    #update data 
    iterm_p = iterm
    err_p = err

    pidterm  = kterm + iterm + dterm

    #saturation 
    if(pidterm < 0) : output = 0
    elif(pidterm >20) :output = 30
    else : output = pidterm

    return output

def self_balancing_pitch(pitch_angle):

    height_front = 0 # calibrated height of the starting position of the robot
    height_rear = 0
    x = 239.6
    #front leg
    new_height_front    = height_front - x/2*np.tan(PID(0,pitch_angle,1,1,1))
    RIGHT_Inverse_Kinematics(front, 0,-l1,new_height_front)
    LEFT_Inverse_Kinematics(front, 0,l1,new_height_front)
    height_front = new_height_front
    #rear leg
    new_height_rear = height_rear + x/2*np.tan(PID(0,pitch_angle,1,1,1))
    RIGHT_Inverse_Kinematics(rear, 0,-l1,new_height_rear)
    LEFT_Inverse_Kinematics(rear, 0,l1,new_height_rear)
    height_rear = new_height_rear


def self_balancing_roll(roll_angle):

    height_right = 0 # calibrated height of the starting position of the robot
    height_left = 0
    x= 80.3
    #right leg
    new_height_right = height_right - x/2*np.tan(PID(0,roll_angle,1,1,1))
    RIGHT_Inverse_Kinematics(front, 0,-l1,new_height_right)
    RIGHT_Inverse_Kinematics(rear, 0,-l1,new_height_right)
    height_right = new_height_right
    #left leg
    new_height_left = height_left + x/2*np.tan(PID(0,roll_angle,1,1,1))
    LEFT_Inverse_Kinematics(front, 0,l1,new_height_left)
    LEFT_Inverse_Kinematics(rear, 0,l1,new_height_left)
    height_left = new_height_left

