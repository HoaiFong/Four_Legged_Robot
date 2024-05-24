from __future__ import division
import time
import math as m
import numpy as np
import array as arr 
import timeit
import threading
import matplotlib
matplotlib.use('Agg')  
import matplotlib.pyplot as plt
# Import the PCA9685 module.


import Adafruit_PCA9685

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()
mpu = Adafruit_PCA9685.MPU6050()

# Configure min and max servo pulse lengths
servo_min = 100  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

###
Roll = 0
Pitch = 0

num_samples = 100
sample_interval = 0.1
timestamps = []
roll = []
pitch = []

def read_sensor_data():
    global roll, pitch
    roll_angle, pitch_angle = mpu.get_RP_Angle()
    roll.append(roll_angle)
    pitch.append(pitch_angle)
    timestamps.append(time.time())

for _ in range(num_samples):
    read_sensor_data()
    time.sleep(sample_interval)

plt.figure(figsize=(10, 6))
plt.plot(timestamps, roll, label='Roll')
plt.plot(timestamps, pitch, label='Pitch')
plt.xlabel('Time')
plt.ylabel('Angle (degrees)')
plt.title('MPU6050 Roll and Pitch over Time')
plt.legend()
plt.grid(True)
plt.savefig('mpu6050_plot.png')

# preTimeGet = 0
# def getRollPitch_MPU():
#     global Roll, Pitch,  # ,preTimeGet
#     Roll, Pitch = mpu.get_RP_Angle()
#     print("Goc truc X: %.1f" %Roll  ,  "Goc truc Y: %.1f" %Pitch)
#     threading.Timer(0.5,getRollPitch_MPU).start()
#     # timeGet = timeit.default_timer()
#     # print(timeGet - preTimeGet, " (seconds)")
#     # preTimeGet = timeGet

l1=46.4 #mm
l2=100 #mm
l3=100 #mm

RH = -130 # mm -> Robot height
SH = 35 # mm -> Swing height

#Define side
Front = 30
Rear = 31

#Define leg adress
front_left = 0
front_right = 3
rear_left = 6
rear_right = 9

# Offset matrix [t1,t2,t3] <=> [0,45,90]
FL = arr.array('i', [90, 27, 12])   # -> <Front Left>
RL = arr.array('i', [98, 70, 13])   # -> <Rear Left>
FR = arr.array('i', [90, 83, 160])  # -> <Front Right>
RR = arr.array('i', [90, 84, 170])  # -> <Front Right>

posFL = np.array([0, 0, 0])   # -> <Front Left>
posRL = np.array([0, 0, 0])   # -> <Rear Left>
posFR = np.array([0, 0, 0])  # -> <Front Right>
posRR = np.array([0, 0, 0])  # -> <Front Right>

# angle to pulse
def angle2pulse(angle):
    return (int)(100+500*angle/180)

# Control leg
def setLegAngles(LegAdress, Theta1, Theta2, Theta3):
    #offset
    if(LegAdress == front_left):
        Theta1 = 180 - (Theta1 + FL[0])
        Theta2 = Theta2 + FL[1]
        Theta3 = Theta3 + FL[2]
    elif(LegAdress == front_right):
        Theta1 = 180 - (Theta1 + FR[0])
        Theta2 = Theta2 + FR[1]
        Theta3 = Theta3 + FR[2]
    elif(LegAdress == rear_left):
        Theta1 = Theta1 + RL[0]
        Theta2 = Theta2 + RL[1]
        Theta3 = Theta3 + RL[2]
    elif(LegAdress == rear_right):
        Theta1 = Theta1 + RR[0]
        Theta2 = Theta2 + RR[1]
        Theta3 = Theta3 + RR[2]
    else: 
        print("Wrong Leg Adress")
        exit()
    # print('Theta1= ',Theta1,'Theta2= ',Theta2,'Theta3= ',Theta3)
    pwm.set_pwm_servo(LegAdress,0,angle2pulse(Theta1),angle2pulse(Theta2),angle2pulse(Theta3))
    
def Calib():
    setLegAngles(front_left, 0, 45, 90)
    setLegAngles(front_right, 0, -45, -90)
    setLegAngles(rear_left, 0, 45, 90)
    setLegAngles(rear_right, 0, -45, -90)
    exit()
    
# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

def Check_work_space(x,y,z):
    k1 = x**2 + y**2 + z**2
    k2 = y**2 + z**2
    if( k2< l1**2 or k1> (l1**2 + (l2 + l3)**2)):
        print("The position :(",x,y,z,") is Out of workspace")
        exit()

def LEFT_Inverse_Kinematics(leg,x,y,z):
    global posFL, posRL, posFR, posRR
    #check work space
    Check_work_space(x,y,z)
    # Theta1
    alpha = m.acos(y/np.sqrt(y**2 + z**2))
    t1 = -(m.acos(l1/np.sqrt(z**2 + y**2)) - alpha)
    c1 = np.cos(t1)
    s1 = np.sin(t1)
    # Theta3
    A = -x
    if(t1!=0):
        B = (l1*c1 - y)/s1
    else:
        B = (-l1*s1 - z)/c1
    c3 = (A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3)
    s3 = np.sqrt(m.fabs(1 - c3**2))
    t3 = m.atan2(s3, c3)
    # Kiem tra dk theta3
    if (t3 < 0):
        print("Error Theta3")
        exit()
    # Theta2
    s2 = (A * (l2 + l3 * c3) + B * l3 * s3)  
    c2 = (B * (l2 + l3 * c3) - A * l3 * s3)  
    t2 = m.atan2(s2, c2)

    # Convert Rad -> Deg
    t1 = t1*180/np.pi
    t2 = t2*180/np.pi
    t3 = t3*180/np.pi

    if(leg == Rear):
        setLegAngles(rear_left,t1,t2,t3)
        posRL = [x, y, z] 
    elif(leg == Front):
        setLegAngles(front_left,t1,t2,t3)
        posFL = [x, y, z] 

def RIGHT_Inverse_Kinematics(leg,x,y,z):
    global posFL, posRL, posFR, posRR
    #check work space
    Check_work_space(x,y,z)
    # Theta1
    alpha = m.asin(y/np.sqrt(y**2 + z**2))
    t1 = -(m.asin(l1/np.sqrt(z**2 + y**2)) + alpha)
    c1 = np.cos(t1)
    s1 = np.sin(t1)
    # Theta3
    A = -x
    if(t1!=0):
        B = (-l1*c1 - y)/s1
    else:
        B = (l1*s1 - z)/c1
    c3 = (A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3)
    s3 = np.sqrt(m.fabs(1 - c3**2))
    t3 = -m.atan2(s3, c3)
    # Kiem tra dk theta3
    if (t3 > 0):
        print("Error Theta3")
        exit()
    # Theta2
    s2 = (A * (l2 + l3 * c3) + B * l3 * s3)  
    c2 = (B * (l2 + l3 * c3) - A * l3 * s3)  
    t2 = -m.atan2(s2, c2)
    
    # Convert Rad -> Deg
    t1 = t1*180/np.pi
    t2 = t2*180/np.pi
    t3 = t3*180/np.pi
    
    if(leg == Rear):
        setLegAngles(rear_right,t1,t2,t3)
        posRR = [x, y, z] 
    elif(leg == Front):
        setLegAngles(front_right,t1,t2,t3)
        posFR = [x, y, z] 
    
def initial_position():
    LEFT_Inverse_Kinematics(Front, 0,l1,RH)
    RIGHT_Inverse_Kinematics(Front, 0,-l1,RH)
    LEFT_Inverse_Kinematics(Rear, 0,l1,RH)
    RIGHT_Inverse_Kinematics(Rear, 0,-l1,RH) 
    time.sleep(2)
    
def all_To_initial(Time_delay):
    FR = posFR
    RR = posRR
    FL = posFL
    RL = posRL
    # for t in np.arange(0.125, 1.005, 0.125):
    for t in np.arange(0.25, 1.005, 0.25):
        RIGHT_Inverse_Kinematics(Front, FR[0] - FR[0]*t, FR[1] - (FR[1]+l1)*t, FR[2] - (FR[2]-RH)*t)
        RIGHT_Inverse_Kinematics(Rear, RR[0] - RR[0]*t, RR[1] - (RR[1]+l1)*t, RR[2] - (RR[2]-RH)*t)
        LEFT_Inverse_Kinematics(Rear, RL[0] - RL[0]*t, RL[1] - (RL[1]-l1)*t, RL[2] - (RL[2]-RH)*t)
        LEFT_Inverse_Kinematics(Front, FL[0] - FL[0]*t, FL[1] - (FL[1]-l1)*t, FL[2] - (FL[2]-RH)*t)
        time.sleep(Time_delay)

def R_P_Y(roll, pitch, yaw):
    pass

pwm.set_pwm_freq(60)
initial_position()
getRollPitch_MPU()
while True:
    exit 
    
    


    
