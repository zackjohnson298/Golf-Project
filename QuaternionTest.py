import serial
import time
import matplotlib.pyplot as plt
from math import *
import keyboard
from VectorMath import *

x = []
frequency = 1800        # Hz
measurementTime = 4     # seconds
rawDataRecieved = []    # Data packets recieved over bluetooth serial
rawData = []            # Reformatted Data Packets
pointsPerPacket = 12    # Number of data points per bluetooth packet
packetSize = 6          # Size of true data packet (3 acc + 3 gyro) = 6
inputPacketSize = 2*pointsPerPacket*packetSize + 4
numberOfRawPackets = packetSize*(frequency*measurementTime-pointsPerPacket)
c = '$'
endList = [c,c,c,c,c];
success = '1'.encode('utf-8')
fail = '0'.encode('utf-8')
maxAttempts = 20

def PrintArray(array):
    output = []
    for ii in range(len(array)):
        output.append(array[ii])
    print(output)

ser = serial.Serial('COM8')
time.sleep(1)
ser.flushInput()
input('Enter any character: ')
ser.write(b'r')
ser.flush()
rawDataRecieved = []    # Data packets recieved over bluetooth serial
rawData = []
sizeArray = []
print("Receiving new swing")
for ii in range(0, numberOfRawPackets, packetSize*pointsPerPacket):
    fullPacket = ser.read(inputPacketSize)
    attempts = 1
    ser.flushInput()
    while ((len(fullPacket) != inputPacketSize) and (attempts < maxAttempts)):
        ser.write(fail)
        ser.flush()
        fullPacket = ser.read(inputPacketSize)
        ser.flushInput()
        attempts = attempts + 1
    ser.write(success)
    ser.flush()
    rawDataRecieved.append(list(fullPacket))

for packet in rawDataRecieved:
    for ii in range(pointsPerPacket):
        index1 = ii*pointsPerPacket + 2
        index2 = index1 + 2*packetSize
        dataLine = list(packet[index1:index2])
        # PrintArray(dataLine)
        rawData.append(list(dataLine))
        sizeArray.append(len(dataLine))

print("Data Points Received: ",len(rawData))
print("Average Packet Size: ",sum(sizeArray)/len(sizeArray))
ser.close()

# Store data into vector arrays
xOffset = -0.004449716846393372
yOffset = 0.005117211423445101
zOffset = -0.002953633863401881
xAcc = []
yAcc = []
zAcc = []
xGyro = []
yGyro = []
zGyro = []
gyroMag = []
# from ADAFRUIT_LSM6DS.cpp line 450
gyroFS = 2000    # For full scale = +/-2000dps
accFS = 16    # For full scale = +/-16G
jj = 0
t = []
acc = [0,0,0]
gyro = [0,0,0]
w = []
dt = 1/frequency
C = pi/180
xsum = 0
ysum = 0
zsum = 0
for packet in rawData:
    acc[0] = (packet[0] << 8 | packet[1])*accFS/(2*16384.0)
    acc[1] = (packet[2] << 8 | packet[3])*accFS/(2*16384.0)
    acc[2] = (packet[4] << 8 | packet[5])*accFS/(2*16384.0)
    for ii in range(0,3):
        if acc[ii] > accFS:
            acc[ii] = acc[ii] - 2*accFS
    gyro[0] = C*(packet[6] << 8 | packet[7])*gyroFS/(2*16384.0)
    gyro[1] = C*(packet[8] << 8 | packet[9])*gyroFS/(2*16384.0)
    gyro[2] = C*(packet[10] << 8 | packet[11])*gyroFS/(2*16384.0)
    for ii in range(0,3):
        if gyro[ii] > C*gyroFS:
            gyro[ii] = gyro[ii] - 2*C*gyroFS
    xAcc.append(acc[0])
    yAcc.append(acc[1])
    zAcc.append(acc[2])
    w.append(Vector(gyro[0]-xOffset,gyro[1]-yOffset,gyro[2]-zOffset))
    gyroMag.append(sqrt(gyro[0]**2 + gyro[1]**2 + gyro[2]**2))
    t.append(float(jj))
    jj = jj + dt
    xsum += gyro[0]
    ysum += gyro[1]
    zsum += gyro[2]

# Convert angular velocities to quaternions
q = [Quaternion(1,0,0,0)]
for ii in range(1,len(t)):
    delta_q = AngularVelocity2Quat(w[ii-1],dt)
    q.append(QuatProd(q[ii-1],delta_q))

for Q in q:
    Q.Normalize()

# Store individual quat components
q0 = []
q1 = []
q2 = []
q3 = []
for ii in range(len(q)):
    Q = Quat2List(q[ii])
    q0.append(Q[0])
    q1.append(Q[1])
    q2.append(Q[2])
    q3.append(Q[3])

# Plot Quaternions
plt.subplot(4,1,1)
plt.plot(t,q0)
plt.subplot(4,1,2)
plt.plot(t,q1)
plt.subplot(4,1,3)
plt.plot(t,q2)
plt.subplot(4,1,4)
plt.plot(t,q3)
plt.show()
