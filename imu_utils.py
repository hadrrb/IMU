import re
import numpy as np
import keyboard
from scipy import signal

# def float_array(str_array):
#     float_array=[]
#     for s in str_array:
#         float_array.append(float(s))
#     return float_array

# def response_to_data(string):
#     delimit = re.compile("Timestamp|Accel|Gyro|Magne|end")
#     str_data = re.split(delimit, string)
    
#     t = int(str_data[1])
#     accel = float_array( str_data[2].split(",") )
#     gyro = float_array( str_data[3].split(",") )
#     magne = float_array( str_data[4].split(",") )
    
#     return np.array([t,accel,gyro,magne])

def response_to_data(string):
    return [float(i) for i in string[2:-5].split(":")]

def is_valid_msg(msg):
    re_float = "[-+]?(\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?"
    re_int = "\d+"
    # re_msg = "Timestamp"+re_int+"Accel"+re_float+","+re_float+","+re_float+"Gyro"+re_float+","+re_float+","+re_float+"Magne"+re_float+","+re_float+","+re_float+"end"
    re_msg = re_float + ":" + re_float + ":" + re_float + ":" + re_float + ":" + re_float + ":" + re_float + ":" + re_float 
    return re.search(re_msg,msg) != None

def wait_for_response(serialport):
    while 1:#refresh response until it is valid
        serialport.flushInput()
        response = str(serialport.readline())
        if is_valid_msg( response ) :
            break
    return response_to_data(response)

def filters(acc, dt):
    #Detect stationary periods
    #accel magnitude
    acc_mag = np.sqrt(acc[:,0]*acc[:,0] + acc[:,1]*acc[:,1] +acc[:,2]*acc[:,2])

    #HP filter accelerometer data
    filtCutOff = 0.001
    b,a = signal.butter(1, (2*filtCutOff)/(1/dt), 'high') #
    acc_magFilt = signal.filtfilt(b, a, acc_mag)

    # Compute absolute value
    acc_magFilt = abs(acc_magFilt)

    # LP filter accelerometer data
    filtCutOff = 5
    [b, a] = signal.butter(1, (2*filtCutOff)/(1/dt), 'low')
    acc_magFilt = signal.filtfilt(b, a, acc_magFilt)

    # Threshold detection
    stationary = acc_magFilt < 2
    
    '''
    % Compute integral drift during non-stationary periods
    velDrift = zeros(size(vel));
    stationaryStart = find([0; diff(stationary)] == -1);
    stationaryEnd = find([0; diff(stationary)] == 1);
    for i = 1:numel(stationaryEnd)
        driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
        enum = 1:(stationaryEnd(i) - stationaryStart(i));
        drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
        velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
    end

    % Remove integral drift
    vel = vel - velDrift;
    '''

    vel = np.zeros_like(acc)
    for i in range (1,acc.shape[0]):
        vel[i, :] = vel[i-1,:] + acc[i,:] * dt
        if(stationary[i] == True):
            vel[i, :] = np.array([0,0,0])

    pos = np.zeros_like(vel)
    for i in range (1,vel.shape[0]):
        pos[i, :] = pos[i-1,:] + vel[i,:] * dt

    return pos