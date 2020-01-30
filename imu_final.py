import serial
from imu_utils import *
import numpy as np
import matplotlib.pyplot as plt
import tkinter

dev = "COM5"
baudrate = 115200
serialport = serial.Serial(dev, baudrate, timeout=1)

accel_lin = np.array([[0,0,0]])
flag = 1 # to create correctly the arrays
pos = np.array([[0,0,0]])
# xa = [x]
# ya = [y]

# try:
plt.ion()
fig = plt.figure()
fig.canvas.set_window_title("Path of the IMU")
plt.title("Path y=f(x)")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
ax = plt.gca()
line1, = ax.plot(pos[:,0], pos[:,1], 'b-')
while True:
    temp = wait_for_response(serialport)
    euler = temp[0:3]
    accel = temp[3:6]
    deltaT = temp[-1]

    rotation_angle = np.array([[np.cos(euler[2]), - np.sin(euler[2]), 0],[np.sin(euler[2]), np.cos(euler[2]), 0],[0, 0 , 1]])
    accel_lin = np.append(accel_lin, [np.matmul(rotation_angle, accel) + [0, 0, 9.8]], axis = 0)
    if flag < 7:
        flag+=1
    else:
        pos = np.append(pos, filters(accel_lin, deltaT), axis = 0)
        flag = 0
        accel_lin = [accel_lin[-1, :]]

    # x += accel_lin[0]*deltaT**2 + accel_lin[0]*deltaT 
    # y += accel_lin[1]*deltaT**2 + accel_lin[1]*deltaT
    # z += accel_lin[2]*deltaT**2 + accel_lin[2]*deltaT

    # xa.append(x)
    # ya.append(y)

    line1.set_xdata(pos[:,0])
    line1.set_ydata(pos[:,1])
    
    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw()
    plt.pause(0.000001)

    print("Pitch = %f, Roll = %f, Yaw = %f"%(temp[0]*180/np.pi,temp[1]*180/np.pi,temp[2]*180/np.pi))
    print("x = %f, y = %f, z = %f"%(pos[-1,0],pos[-1,1],pos[-1,2]))


# except:
#     print("Program is going to stop")
