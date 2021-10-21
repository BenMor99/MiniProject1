import time
import serial
import numpy as np
from vpython import *

arduinoData = serial.Serial('com8', 115200)
time.sleep(1)

toRad = np.pi/180.0
toDeg = 1/toRad

# Simulate your object using VPython

while True:
    while arduinoData.inWaiting() == 0:
        pass
    dataPacket = arduinoData.readline()
    try:
        dataPacket = str(dataPacket, 'utf-8')
        splitPacket = dataPacket.split(",")
        roll = float(splitPacket[0])*toRad
        pitch = float(splitPacket[1])*toRad
        yaw = float(splitPacket[2])*toRad
        print(roll*toDeg, pitch*toDeg, yaw*toDeg)
        
        # Change the attributes of your object to syncronize it with real time motions

    except:
        pass