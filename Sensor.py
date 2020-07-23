from VectorMath import *
from GPF_Constants import *
from math import *
import serial
import time

#-----------------------------------------------------------------------------#
#The Sensor Class included all the methods related to the sensor object. This
#should include retrieving the sensor data through bluetooth, processing the
#data, and rotating sensor data into the world frame for further analysis down
#stream.
#-----------------------------------------------------------------------------#

class Sensor():
    def __init__(self):
        self.rawSensorBytes = []
        self.acc = []
        self.accWorld = []
        self.gyro = []
        self.gyroWorld = []
        self.accMag = []
        self.gyroMag = []
        self.velWorld = []
        self.quat = []

    # def GetRawData(self):
    #     self.GetSensorBytes()
    #     self.ConvertRawData()

    #GetSensorBytes() is responsible for opening the bluetooth serial port and
    #requesting packet package. It is also includes error correction to ensure
    #no data packet is missing.
    def GetSensorBytes(self):
        ser = serial.Serial('COM8')
        time.sleep(1)
        ser.flushInput()
        input('Enter any character: ') #Comment out later
        ser.write(b'r')
        ser.flush()
        rawDataRecieved = []    # Data packets recieved over bluetooth serial
        maxAttempts = 20
        fail = '0'.encode('utf-8')
        success = '1'.encode('utf-8')
        for ii in range(0, NUMBER_OF_RAW_PACKETS, PACKET_SIZE*POINTS_PER_PACKET):
            fullPacket = ser.read(INPUT_PACKET_SIZE)
            attempts = 1
            ser.flushInput()
            while ((len(fullPacket) != INPUT_PACKET_SIZE) and (attempts < maxAttempts)):
                ser.write(fail)
                ser.flush()
                fullPacket = ser.read(INPUT_PACKET_SIZE)
                ser.flushInput()
                attempts = attempts + 1
            ser.write(success)
            ser.flush()
            rawDataRecieved.append(list(fullPacket))

        # Extracts individual datapoints from packets
        for packet in rawDataRecieved:
            for ii in range(POINTS_PER_PACKET):
                index1 = ii*POINTS_PER_PACKET + 2
                index2 = index1 + 2*PACKET_SIZE
                dataLine = list(packet[index1:index2])
                self.rawSensorBytes.append(list(dataLine))
        ser.close()


    #ConvertRawData() is responsible for bit shifting and magnitude correction
    #to ensure the sensor outputs are within their full scale value.
    def ConvertRawData(self):
        jj = 0
        x = []
        acc = [0,0,0]
        gyro = [0,0,0]
        for packet in self.rawSensorBytes:
            acc[0] = (packet[0] << 8 | packet[1])*ACC_FS/(2*16384.0)
            acc[1] = (packet[2] << 8 | packet[3])*ACC_FS/(2*16384.0)
            acc[2] = (packet[4] << 8 | packet[5])*ACC_FS/(2*16384.0)
            for ii in range(0,3):
                if acc[ii] > ACC_FS:
                    acc[ii] = acc[ii] - 2*ACC_FS
            gyro[0] = DEG2RAD*(packet[6] << 8 | packet[7])*GYRO_FS/(2*16384.0)
            gyro[1] = DEG2RAD*(packet[8] << 8 | packet[9])*GYRO_FS/(2*16384.0)
            gyro[2] = DEG2RAD*(packet[10] << 8 | packet[11])*GYRO_FS/(2*16384.0)
            for ii in range(0,3):
                if gyro[ii] > DEG2RAD*GYRO_FS:
                    gyro[ii] = gyro[ii] - 2*DEG2RAD*GYRO_FS
            self.acc.append(Vector(acc[0],acc[1],acc[2]))
            self.gyro.append(Vector(gyro[0],gyro[1],gyro[2]))
            self.accMag.append(sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2))


    #OffsetGyroBias() is responsible for offsetting the gyro's outputs by a
    #constant to limit drift in the quaternion later.
    def OffsetGyroBias(self):
        xOffset = -0.004449716846393372
        yOffset = 0.005117211423445101
        zOffset = -0.002953633863401881
        offset = Vector(xOffset,yOffset,zOffset)
        for ii in range(len(self.gyro)):
            newGyro = VectorSubtract(self.gyro[ii],offset)
            self.gyro[ii] = newGyro
            self.gyroMag.append(Norm(newGyro))


    #SensorToBody() is responsible for rotating the sensor frame into the body
    #frame of the golf club. This is just incase the sensor board is aligned
    #differently from the golf club frame when mounted.
    def SensorToBody(self):
        # Quaternion to rotate from actual sensor frame to club-fixed body frame
        qS_B = Quaternion(1/sqrt(2),0,-1/sqrt(2),0)
        # qS_B = Quaternion()
        for ii in range(len(self.acc)):
            aS = self.acc[ii]
            gS = self.gyro[ii]
            aB = QuatRot(qS_B,aS)
            gB = QuatRot(qS_B,gS)
            self.acc[ii] = aB
            self.gyro[ii] = gB

    # FIX LATER!!!!!!!!!!!!!
    #DefineWorldFrame() is responsible for establishing the world frame such
    #that data can be outputted with respect to a fixed frame.
    def DefineWorldFrame(self):
        calTime = 0.5
        rxSum = 0
        rySum = 0
        rzSum = 0
        for ii in range(int(calTime*FREQUENCY)):
            aS = self.acc[ii]
            rxSum += aS.x
            rySum += aS.y
            rzSum += aS.z
        rx = rxSum / (calTime*FREQUENCY)
        ry = rySum / (calTime*FREQUENCY)
        rz = rzSum / (calTime*FREQUENCY)
        r = Vector(rx,ry,rz)
        r.Normalize()
        yS = Vector(0,1,0)
        zS = Vector(0,0,1)
        xW = Cross(yS,r)
        xW.Normalize()
        theta1 = acos(Dot(yS,r)) - pi/2
        q1_0 = cos(theta1/2)
        q1_1 = xW.x*sin(theta1/2)
        q1_2 = xW.y*sin(theta1/2)
        q1_3 = xW.z*sin(theta1/2)
        q1 = Quaternion(q1_0, q1_1, q1_2, q1_3)
        r2 = QuatRot(q1,r)
        y2 = QuatRot(q1,yS)
        z2 = QuatRot(q1,zS)
        theta2 = acos(Dot(z2,r))
        q2_0 = cos(theta2/2)
        q2_1 = y2.x*sin(theta2/2)
        q2_2 = y2.y*sin(theta2/2)
        q2_3 = y2.z*sin(theta2/2)
        q2 = Quaternion(q2_0, q2_1, q2_2, q2_3)
        self.quat.append(QuatConj(QuatProd(q1,q2)))

    #GetOrientation() is responsible for getting the quaternion from the gyro
    #data.
    def GetOrientation(self):
        dt = 1 / FREQUENCY
        for ii in range(1,len(self.acc)):
            w = self.gyro[ii-1]
            delta_q = AngularVelocity2Quat(w,dt)
            self.quat.append(QuatProd(self.quat[ii-1],delta_q))
            self.quat[ii].Normalize()

    #BodyToWorld() is responsible for rotating sensor data in the body frame to
    #the fixed world frame.
    def BodyToWorld(self):
        for ii in range(len(self.acc)):
            aB = self.acc[ii]
            gB = self.gyro[ii]
            aWorld = QuatRot(self.quat[ii],aB)
            gWorld = QuatRot(self.quat[ii],gB)
            self.accWorld.append(aWorld)
            self.gyroWorld.append(gWorld)

    #GetVelocity() is responsible for getting the velocity of the sensor in the
    #world frame. This will be used later for velocities of the club head.
    def GetVelocity(self):
        stationary = []
        for wMag in self.gyroMag:
            if wMag <= GYRO_THRESH:
                stationary.append(1)
            else:
                stationary.append(0)

        xddot = []
        yddot = []
        zddot = []

        for ii in range(0,len(self.acc)):
            a = Vec2List(self.accWorld[ii])
            xddot.append(a[0])
            yddot.append(a[1])
            zddot.append(a[2])

        nonStatPeriods = []
        xdot = [0]
        ydot = [0]
        zdot = [0]
        startIndex = 0
        for ii in range(1,len(self.acc)):
            if stationary[ii] == 1:
                xdot.append(0)
                ydot.append(0)
                zdot.append(0)
                if stationary[ii-1] == 0:
                    nonStatPeriods.append(list([startIndex,ii-1]))
            else:
                xdot.append(GRAVITY*xddot[ii]/FREQUENCY + xdot[ii-1])
                ydot.append(GRAVITY*yddot[ii]/FREQUENCY + ydot[ii-1])
                zdot.append(GRAVITY*zddot[ii]/FREQUENCY + zdot[ii-1])
                if stationary[ii-1] == 1:
                    startIndex = ii-1

        for i in range(0,len(nonStatPeriods)):
            i1 = nonStatPeriods[i][0]
            i2 = nonStatPeriods[i][1]
            N = i2 - i1
            dvx = xdot[i2]-xdot[i1]
            dvy = ydot[i2]-ydot[i1]
            dvz = zdot[i2]-zdot[i1]
            ratex = dvx/N
            ratey = dvy/N
            ratez = dvz/N
            for ii in range(i1,i2+1):
                xdot[ii] -= ratex*(ii - i1)
                ydot[ii] -= ratey*(ii - i1)
                zdot[ii] -= ratez*(ii - i1)

        for ii in range(0,len(self.acc)):
            v = Vector(xdot[ii],ydot[ii],zdot[ii])
            self.velWorld.append(v)
