from __future__ import division
import time
from time import sleep
import math as m
import numpy as np
import array as arr 
# Import the PCA9685 module.

import Adafruit_PCA9685

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
mpu = Adafruit_PCA9685.MPU6050()

while True:        
    # m,n,p = mpu.get_Gyro()
    # a,b,c = mpu.get_RP_Angle()
    # t = mpu.get_Temp()
    roll, pitch = mpu.get_RP_Angle()
    print("Goc truc X: %.1f" %roll  ,  "Goc truc Y: %.1f" %pitch)
    sleep(0.04)
