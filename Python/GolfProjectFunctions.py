import serial
import time
import matplotlib.pyplot as plt
import numpy as np
import keyboard
from math import *
from scipy.fft import fft
from scipy.signal import butter, lfilter, filtfilt
from statsmodels.nonparametric.smoothers_lowess import lowess
from QuaternionMath import *
from MatMath import *

class Frame():

    def __init__(self,frameType):
        self.type = frameType
        self.q0 = []
        self.q1 = []
        self.q2 = []
        self.q3 = []
        self.gyroX = []
        self.gyroY = []
        self.gyroZ = []
        self.xddot = []
        self.yddot = []
        self.zddot = []
        self.t = []
        self.x = [0,0]
        self.y = [0,0]
        self.z = [0,0]
        self.xdot = [0]
        self.ydot = [0]
        self.zdot = [0]
        self.xdotCH = []
        self.ydotCH = []
        self.zdotCH = []
        self.Clubspeed = []
        self.Err = [0,0,0]
        self.pltLabels = {}
        self.pltLabels['xlabel'] = 'Time t [s]'
        self.pltLabels['Position_ylabels'] = ['x [m/s^2]','y [m/s^2]','z [m/s^2]']
        self.pltLabels['Velocity_ylabels'] = ['xdot [m/s^2]','ydot [m/s^2]','zdot [m/s^2]']
        self.pltLabels['Acceleration_ylabels'] = ['xddot [m/s^2]','yddot [m/s^2]','zddot [m/s^2]']
        self.pltLabels['Gyro_ylabels'] = ['GyroX [rad/s]','GyroY [rad/s]','GyroZ [rad/s]']
        self.pltLabels['Club_ylabels'] = ['xdot [mph]','ydot [mph]','zdot [mph]','Magnitude [mph]']
        self.pltLabels['Title'] = frameType + 'Frame'
        self.pltLabels['Quaternion_ylabels'] = ['q0','q1','q2','q3']

    def GetVelZUPT(self, gravity = 9.81, thresh=0.035, lowCutoff = 10, highCutoff = 0.001, showPlots = False):
        aMag = []
        stationary = []
        for ii in range(0,len(self.t)):
            aMag.append(sqrt(self.xddot[ii]**2 + self.yddot[ii]**2 + self.zddot[ii]**2))
        aMag = abs(ButterFilter(aMag,self.t,highCutoff,1,'high'))
        aMag = ButterFilter(aMag,self.t,lowCutoff,1,'low')
        for ii in range(0,len(aMag)):
            if abs(aMag[ii]) < thresh:
                stationary.append(1)
            else:
                stationary.append(0)
        if showPlots:
            plt.figure()
            plt.plot(self.t,stationary)
            plt.plot(self.t,aMag)
            plt.title('Acceleration Magnitude')

        xddot = ButterFilter(self.xddot,self.t,highCutoff,1,'high')
        yddot = ButterFilter(self.yddot,self.t,highCutoff,1,'high')
        zddot = ButterFilter(self.zddot,self.t,highCutoff,1,'high')

        nonStatPeriods = []
        for ii in range(1,len(self.t)):
            if stationary[ii] == 1:
                self.xdot.append(0)
                self.ydot.append(0)
                self.zdot.append(0)
                if stationary[ii-1] == 0:
                    nonStatPeriods.append(list([startIndex,ii-1]))
            else:
                dt = self.t[ii] - self.t[ii-1]
                self.xdot.append(gravity*xddot[ii]*dt + self.xdot[ii-1])
                self.ydot.append(gravity*yddot[ii]*dt + self.ydot[ii-1])
                self.zdot.append(gravity*zddot[ii]*dt + self.zdot[ii-1])
                if stationary[ii-1] == 1:
                    startIndex = ii-1
        if showPlots:
            plt.figure()
            plt.plot(self.t,self.xdot)
            plt.plot(self.t,self.ydot)
            plt.plot(self.t,self.zdot)
            plt.legend(('xdot','ydot','zdot'))
            plt.title('Raw Velocities')
        for i in range(0,len(nonStatPeriods)):
            i1 = nonStatPeriods[i][0]
            i2 = nonStatPeriods[i][1]
            N = i2 - i1
            dvx = self.xdot[i2]-self.xdot[i1]
            dvy = self.ydot[i2]-self.ydot[i1]
            dvz = self.zdot[i2]-self.zdot[i1]
            ratex = dvx/N
            ratey = dvy/N
            ratez = dvz/N
            for ii in range(i1,i2+1):
                self.xdot[ii] -= ratex*(ii - i1)
                self.ydot[ii] -= ratey*(ii - i1)
                self.zdot[ii] -= ratez*(ii - i1)
        if showPlots:
            plt.figure()
            plt.plot(self.t,self.xdot)
            plt.plot(self.t,self.ydot)
            plt.plot(self.t,self.zdot)
            plt.legend(('xdot','ydot','zdot'))
            plt.title('Compensated Velocities')
            plt.show()

    def GetVelocity(self):
        t = self.t
        # xdot = self.xddot
        # ydot = self.yddot
        # zdot = self.zddot
        xddot = self.xddot
        yddot = self.yddot
        zddot = self.zddot
        for ii in range(1,len(t)):
            dt = t[ii] - t[ii-1]
            self.xdot.append(xddot[ii]*dt + self.xdot[ii-1])
            self.ydot.append(yddot[ii]*dt + self.ydot[ii-1])
            self.zdot.append(zddot[ii]*dt + self.zdot[ii-1])

    def GetPosition(self):
        t = self.t
        xdot = self.xdot
        ydot = self.ydot
        zdot = self.zdot
        for ii in range(2,len(t)):
            dt = t[ii] - t[ii-1]
            self.x.append(xdot[ii]*dt + self.x[ii-1])
            self.y.append(ydot[ii]*dt + self.y[ii-1])
            self.z.append(zdot[ii]*dt + self.z[ii-1])

    def FillData(self, data):
        if self.type == 'Sensor':
            for ii in range(0,len(data)):
                q = Normalize(data[ii][0:4])
                self.q0.append(data[ii][0])
                self.q1.append(data[ii][1])
                self.q2.append(data[ii][2])
                self.q3.append(data[ii][3])
                self.gyroX.append(data[ii][4])
                self.gyroY.append(data[ii][5])
                self.gyroZ.append(data[ii][6])
                self.xddot.append(data[ii][7])
                self.yddot.append(data[ii][8])
                self.zddot.append(data[ii][9])
                self.t.append(data[ii][10])
        elif self.type == 'World':
            for ii in range(0,len(data)):
                q = Normalize(data[ii][0:4])
                vS = data[ii][7:10]
                vW = Qrotate(q,vS)
                self.q0.append(q[0])
                self.q1.append(q[1])
                self.q2.append(q[2])
                self.q3.append(q[3])
                self.gyroX.append(data[ii][4])
                self.gyroY.append(data[ii][5])
                self.gyroZ.append(data[ii][6])
                self.xddot.append(vW[0])
                self.yddot.append(vW[1])
                self.zddot.append(vW[2])
                self.t.append(data[ii][10])

    def HardcodeBS(self,threshold):
        for ii in range(0,len(self.t)):
            magnitude = sqrt(self.xddot[ii]**2 + self.yddot[ii]**2 + self.zddot[ii]**2)
            if magnitude <= threshold:
                self.xddot[ii] = 0
                self.yddot[ii] = 0
                self.zddot[ii] = 0

    def OffsetPosition(self,t0,t1,t2):
        T = np.matrix([[t0**2,t0,1],[t1**2,t1,1],[t2**2,t2,1]])
        count0 = 0
        count1 = 0
        count2 = 0

        for ii in range(0,len(self.t)):
            if self.t[ii] >= t0:
                count0 = ii
                break
        for ii in range(0,len(self.t)):
            if self.t[ii] >= t1:
                count1 = ii
                break
        for ii in range(0,len(self.t)):
            if self.t[ii] >= t2:
                count2 = ii
                break
        x0 = self.x[count0]
        x1 = self.x[count1]
        x2 = self.x[count2]
        y0 = self.y[count0]
        y1 = self.y[count1]
        y2 = self.y[count2]
        z0 = self.z[count0]
        z1 = self.z[count1]
        z2 = self.z[count2]

        X = np.matrix([[x0],[x1],[x2]])
        Y = np.matrix([[y0],[y1],[y2]])
        Z = np.matrix([[z0],[z1],[z2]])

        Cx = np.linalg.inv(T)*X
        Cy = np.linalg.inv(T)*Y
        Cz = np.linalg.inv(T)*Z

        for ii in range(0,len(self.t)):

            self.x[ii] -= float(Cx[0]*self.t[ii]**2 + Cx[1]*self.t[ii] + Cx[2])
            self.y[ii] -= float(Cy[0]*self.t[ii]**2 + Cy[1]*self.t[ii] + Cy[2])
            self.z[ii] -= float(Cz[0]*self.t[ii]**2 + Cz[1]*self.t[ii] + Cz[2])

    def Calibrate(self,calibrationTime):
        count = 0
        for ii in range(10,len(self.t)):
            if self.t[ii] >= calibrationTime:
                break
            count = count + 1
        self.Err[0] = sum(self.xddot[10:count])/len(self.xddot[10:count])
        self.Err[1] = sum(self.yddot[10:count])/len(self.yddot[10:count])
        self.Err[2] = sum(self.zddot[10:count])/len(self.zddot[10:count])

        for ii in range(0,len(self.t)):
            self.xddot[ii] -= self.Err[0]
            self.yddot[ii] -= self.Err[1]
            self.zddot[ii] -= self.Err[2]

    def GetGravityMag(self,calibrationTime):
        gravityList = []
        for ii in range(0,len(self.t)):
            if self.t[ii] <= calibrationTime:
                gravityList.append(sqrt(self.xddot[ii]**2 + self.yddot[ii]**2 + self.zddot[ii]**2))
            else:
                break
        return sum(gravityList)/len(gravityList)

    def LowessFilter(self,frac):
        fx = lowess(self.xddot,self.t,frac)
        fy = lowess(self.yddot,self.t,frac)
        fz = lowess(self.zddot,self.t,frac)
        self.xddot = fx[:,1]
        self.yddot = fy[:,1]
        self.zddot = fz[:,1]

    def GetClubheadSpeedMat(self,L,highCutoff = 0.001):
        Vc   = np.matrix([[0],[0],[0]])      # Clubhead velocity in world frame [m/s]
        Vs   = np.matrix([[0],[0],[0]])      # Sensor velocity in world frame [m/s]
        T    = np.matrix([0,0,0],[0,0,0],[0,0,0])
        w    = [0,0,0]    # Angular Velocity in sensor frame [rad/s]
        club = [0,0,-L]   # Location of clubhead w.r.t sensor in sensor frame [m]
        wx = ButterFilter(self.gyroX,self.t,highCutoff,1,'high')
        wy = ButterFilter(self.gyroY,self.t,highCutoff,1,'high')
        wz = ButterFilter(self.gyroZ,self.t,highCutoff,1,'high')

        for ii in range(0,len(self.t)):
            Vs[0] = self.xdot[ii]
            Vs[1] = self.ydot[ii]
            Vs[2] = self.zdot[ii]
            q[0]  = self.q0[ii]
            q[1]  = self.q1[ii]
            q[2]  = self.q2[ii]
            q[3]  = self.q3[ii]
            w[0]  = self.gyroX[ii]
            w[1]  = self.gyroY[ii]
            w[2]  = self.gyroZ[ii]
            R = Quat2Mat(q)
            Vc = 2.23694*(Vs + T*Skew(w)*club)
            self.xdotCH.append(Vc[0])
            self.ydotCH.append(Vc[1])
            self.zdotCH.append(Vc[2])
            self.Clubspeed.append(sqrt(Vc[0]**2 + Vc[1]**2 + Vc[2]**2))

    def GetClubheadSpeedQuat(self,L,highCutoff = 0.001):
        Vc   = np.array([0,0,0])      # Clubhead velocity in world frame [m/s]
        Vs   = np.array([0,0,0])      # Sensor velocity in world frame [m/s]
        w    = [0,0,0,0]    # Angular Velocity in sensor frame [0,x,y,z][rad/s]
        q    = [0,0,0,0]    # Quaternion between Sensor frame and World Frame
        qdot = [0,0,0,0]    # Quaternion Velocities
        club = [0,0,0,-L]   # Location of clubhead w.r.t sensor in sensor frame [0,x,y,z]
        wx = ButterFilter(self.gyroX,self.t,highCutoff,1,'high')
        wy = ButterFilter(self.gyroY,self.t,highCutoff,1,'high')
        wz = ButterFilter(self.gyroZ,self.t,highCutoff,1,'high')

        for ii in range(0,len(self.t)):
            Vs[0] = self.xdot[ii]
            Vs[1] = self.ydot[ii]
            Vs[2] = self.zdot[ii]
            q[0]  = self.q0[ii]
            q[1]  = self.q1[ii]
            q[2]  = self.q2[ii]
            q[3]  = self.q3[ii]
            w[1]  = self.gyroX[ii]
            w[2]  = self.gyroY[ii]
            w[3]  = self.gyroZ[ii]
            qdot = .5*np.array(Qproduct(q,w))
            array1 = np.array(Qproduct(Qproduct(qdot,club),Qconj(q)))
            array2 = np.array(Qproduct(Qproduct(q,club),Qconj(qdot)))
            Vc = 2.23694*(Vs + array1[1:4] + array2[1:4])
            self.xdotCH.append(Vc[0])
            self.ydotCH.append(Vc[1])
            self.zdotCH.append(Vc[2])
            self.Clubspeed.append(sqrt(Vc[0]**2 + Vc[1]**2 + Vc[2]**2))

    def PlotData(self,values = ['Position','Velocity','Acceleration','Quaternion','Gyro','Club']):
        t = self.t
        labels = self.pltLabels
        for ii in range(0,len(values)):
            if values[ii] == 'Position':
                data = [self.x,self.y,self.z]
                ncols = 1
                nrows = 3
            if values[ii] == 'Velocity':
                data = [self.xdot,self.ydot,self.zdot]
                ncols = 1
                nrows = 3
            if values[ii] == 'Acceleration':
                data = [self.xddot,self.yddot,self.zddot]
                ncols = 1
                nrows = 3
            if values[ii] == 'Quaternion':
                data = [self.q0,self.q1,self.q2,self.q3]
                ncols = 1
                nrows = 4
            if values[ii] == 'Gyro':
                data = [self.gyroX,self.gyroY,self.gyroZ]
                print(len(self.gyroX))
                ncols = 1
                nrows = 3
            if values[ii] == 'Club':
                data = [self.xdotCH,self.ydotCH,self.zdotCH,self.Clubspeed]
                ncols = 1
                nrows = 3
            plt.figure()
            plt.suptitle(labels['Title'] + ' ' + values[ii])
            for jj in range(0,len(data)):
                if values[ii] == 'Club' and jj == 3:
                    plt.figure()
                    plt.title(labels['Title'] + ' ' + values[ii])
                else:
                    plt.subplot(nrows,ncols,jj+1)
                plt.plot(t,data[jj])
                plt.xlabel(labels['xlabel'])
                plt.ylabel(labels[values[ii] + '_ylabels'][jj])
        plt.show()

    def PlotFrequencyDomain(self,values = ['Acceleration','Gyro']):
        N = len(self.t)
        dt = self.t[-1]/N
        for ii in range(0,len(values)):
            if values[ii] == 'Acceleration':
                xdata = np.array(self.xddot)
                ydata = np.array(self.yddot)
                zdata = np.array(self.zddot)
                xf = np.linspace(0.0, 1.0/(2.0*dt), N//2)
                xdataf = fft(xdata)
                ydataf = fft(ydata)
                zdataf = fft(zdata)
                xdataf = 2.0/N * np.abs(xdataf[0:N//2])
                ydataf = 2.0/N * np.abs(ydataf[0:N//2])
                zdataf = 2.0/N * np.abs(zdataf[0:N//2])
                plt.figure()
                plt.suptitle(values[ii])
                plt.subplot(311)
                plt.plot(xf,xdataf)
                plt.subplot(312)
                plt.plot(xf,ydataf)
                plt.subplot(313)
                plt.plot(xf,zdataf)
            if values[ii] == 'Gyro':
                xdata = np.array(self.gyroX)
                ydata = np.array(self.gyroY)
                zdata = np.array(self.gyroZ)
                xf = np.linspace(0.0, 1.0/(2.0*dt), N//2)
                xdataf = fft(xdata)
                ydataf = fft(ydata)
                zdataf = fft(zdata)
                xdataf = 2.0/N * np.abs(xdataf[0:N//2])
                ydataf = 2.0/N * np.abs(ydataf[0:N//2])
                zdataf = 2.0/N * np.abs(zdataf[0:N//2])
                plt.figure()
                plt.suptitle(values[ii])
                plt.subplot(311)
                plt.plot(xf,xdataf)
                plt.subplot(312)
                plt.plot(xf,ydataf)
                plt.subplot(313)
                plt.plot(xf,zdataf)
                plt.show()

def Normalize(v):
    sum = 0
    vNew = []
    for ii in range(0,len(v)):
    	sum = sum + v[ii]**2
    mag = sqrt(sum)
    for ii in range(0,len(v)):
    	vNew.append(v[ii]/mag)
    return vNew

def ButterFilter(data,t,cutoff,order,btype):
    def butter_lowpass(cutoff, fs, order,btype):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype, analog=False)
        return b, a

    def butter_lowpass_filter(data, cutoff, fs, order,btype):
        b, a = butter_lowpass(cutoff, fs, order, btype)
        y = filtfilt(b, a, data)
        return y

    N = len(t)
    fs = N/t[-1]

    return butter_lowpass_filter(np.array(data), cutoff, fs, order,btype)

def TrimData(data,trimTime):
    count = 0
    for ii in range(0,len(data)):
        if data[ii][10] >= trimTime:
            break
        else:
            count = count + 1
    newData = list(data[count:len(data)])
    for ii in range(0,len(newData)):
        newData[ii][10] -= trimTime
    return newData

def GetRawData(port,baudrate,BUFFER_LEN):
    # set up the serial line
    ser = serial.Serial(port, baudrate)
    time.sleep(2)
    ser.write(b'r')

    #Remove initialization instructions from Serial Port
    for i in range(0,16):
        b = ser.readline()         # read a byte string

    # Read and record the data
    rawData = []
    ii = 0
    while True:
        if keyboard.is_pressed('q'):
            print('Keyboard Interupt')
            break
        else:
            b = ser.readline()
            if len(b) == BUFFER_LEN:
                if ii == 0:
                    start = time.time()
                    t = [0]
                else:
                    t = [time.time() - start]
                rawData.append(list(b) + list(t))
                ii = ii + 1

    ser.close()
    return rawData

def ConvertRawData(rawData,gravity,GYRO_FS):
    data = []
    dataRow = [0,0,0,0,0,0,0,0,0,0,0]
    for ii in range(0,len(rawData)):
        b = rawData[ii]
        # Quaternions
        dataRow[0] = (b[2] << 24 | b[3] << 16 | b[4] << 8 | b[5])/1073741824.0
        dataRow[1] = (b[6] << 24 | b[7] << 16 | b[8] << 8 | b[9])/1073741824.0
        dataRow[2] = (b[10] << 24 | b[11] << 16 | b[12] << 8 | b[13])/1073741824.0
        dataRow[3] = (b[14] << 24 | b[15] << 16 | b[16] << 8 | b[17])/1073741824.0
        for jj in range(0,4):
            if dataRow[jj] >= 2:
                dataRow[jj] = dataRow[jj] - 4
        # Gyro [rad/s]
        dataRow[4] = (pi/180)*(b[18] << 24 | b[19] << 16 | b[20] << 8 | b[21])/536870.912
        dataRow[5] = (pi/180)*(b[22] << 24 | b[23] << 16 | b[24] << 8 | b[25])/536870.912
        dataRow[6] = (pi/180)*(b[26] << 24 | b[27] << 16 | b[28] << 8 | b[29])/536870.912
        for jj in range(4,7):
            if dataRow[jj] >= (pi/180)*4000:
                dataRow[jj] = dataRow[jj] - (pi/180)*8000
        # Accelerometer [G]
        dataRow[7] = (b[30] << 24 | b[31] << 16 | b[32] << 8 | b[33])/536870912.0
        dataRow[8] = (b[34] << 24 | b[35] << 16 | b[36] << 8 | b[37])/536870912.0
        dataRow[9] = (b[38] << 24 | b[39] << 16 | b[40] << 8 | b[41])/536870912.0
        for jj in range(7,10):
            if dataRow[jj] >= gravity*4:
                dataRow[jj] = dataRow[jj] - gravity*8
        # Time [s]
        dataRow[10] = b[44]
        data.append(dataRow.copy())
    return data
