import serial
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.fft import fft
from scipy.signal import butter, lfilter
from statsmodels.nonparametric.smoothers_lowess import lowess
from QuaternionMath import *

class Frame():

    def __init__(self,frameType):
        self.type = frameType
        self.q0 = []
        self.q1 = []
        self.q2 = []
        self.q3 = []
        self.x = [0,0]
        self.y = [0,0]
        self.z = [0,0]
        self.xdot = [0]
        self.ydot = [0]
        self.zdot = [0]
        self.xddot = []
        self.yddot = []
        self.zddot = []
        self.Err = [0,0,0]
        self.t = []
        self.pltLabels = {}
        self.pltLabels['xlabel'] = 'Time t [s]'
        self.pltLabels['Accel_ylabels'] = ['xddot [m/s^2]','yddot [m/s^2]','zddot [m/s^2]']
        self.pltLabels['Accel_title'] = frameType + 'Frame Accelerations'
        self.pltLabels['Quat_ylabels'] = ['q0','q1','q2','q3']
        self.pltLabels['Quat_title'] = frameType + 'Frame Quaternions'

    def GetMotion(self):
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
            if ii >= 2:
                self.x.append(self.xdot[ii]*dt + self.x[ii-1])
                self.y.append(self.ydot[ii]*dt + self.y[ii-1])
                self.z.append(self.zdot[ii]*dt + self.z[ii-1])



    def FillData(self, data):
        if self.type == 'Sensor':
            for ii in range(0,len(data)):
                self.q0.append(data[ii][0])
                self.q1.append(data[ii][1])
                self.q2.append(data[ii][2])
                self.q3.append(data[ii][3])
                self.xddot.append(data[ii][4])
                self.yddot.append(data[ii][5])
                self.zddot.append(data[ii][6])
                self.t.append(data[ii][7])
        elif self.type == 'World':
            for ii in range(0,len(data)):
                q = data[ii][0:4]
                vS = data[ii][4:7]
                vW = Qrotate(q,vS)
                self.q0.append(q[0])
                self.q1.append(q[1])
                self.q2.append(q[2])
                self.q3.append(q[3])
                self.xddot.append(vW[0])
                self.yddot.append(vW[1])
                self.zddot.append(vW[2])
                self.t.append(data[ii][7])

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

    def LowessFilter(self,frac):
        fx = lowess(self.xddot,self.t,frac)
        fy = lowess(self.yddot,self.t,frac)
        fz = lowess(self.zddot,self.t,frac)
        self.xddot = fx[:,1]
        self.yddot = fy[:,1]
        self.zddot = fz[:,1]

    def ButterFilter(self,cutoff,order):
        def butter_lowpass(cutoff, fs, order=5):
            nyq = 0.5 * fs
            normal_cutoff = cutoff / nyq
            b, a = butter(order, normal_cutoff, btype='low', analog=False)
            return b, a

        def butter_lowpass_filter(data, cutoff, fs, order=5):
            b, a = butter_lowpass(cutoff, fs, order=order)
            y = lfilter(b, a, data)
            return y

        N = len(self.t)
        fs = N/self.t[-1]

        self.xddot = butter_lowpass_filter(np.array(self.xddot), cutoff[0], fs, order)
        self.yddot = butter_lowpass_filter(np.array(self.yddot), cutoff[1], fs, order)
        self.zddot = butter_lowpass_filter(np.array(self.zddot), cutoff[2], fs, order)

    def PlotData(self,values = ['acc','quat']):
        for ii in range(0,len(values)):
            if values[ii] == 'pos':
                labels = self.pltLabels
                plt.figure()
                plt.suptitle(labels['Accel_title'])
                plt.subplot(311)
                plt.plot(self.t,self.x)
                plt.xlabel(labels['xlabel'])
                plt.ylabel(labels['Accel_ylabels'][0])
                plt.subplot(312)
                plt.plot(self.t,self.y)
                plt.xlabel(labels['xlabel'])
                plt.ylabel(labels['Accel_ylabels'][1])
                plt.subplot(313)
                plt.plot(self.t,self.z)
                plt.xlabel(labels['xlabel'])
                plt.ylabel(labels['Accel_ylabels'][2])
            if values[ii] == 'acc':
                labels = self.pltLabels
                plt.figure()
                plt.suptitle(labels['Accel_title'])
                plt.subplot(311)
                plt.plot(self.t,self.xddot)
                plt.xlabel(labels['xlabel'])
                plt.ylabel(labels['Accel_ylabels'][0])
                plt.subplot(312)
                plt.plot(self.t,self.yddot)
                plt.xlabel(labels['xlabel'])
                plt.ylabel(labels['Accel_ylabels'][1])
                plt.subplot(313)
                plt.plot(self.t,self.zddot)
                plt.xlabel(labels['xlabel'])
                plt.ylabel(labels['Accel_ylabels'][2])
            if values[ii] == 'quat':
                labels = self.pltLabels
                plt.figure()
                plt.suptitle(labels['Quat_title'])
                plt.subplot(411)
                plt.plot(self.t,self.q0)
                plt.xlabel(labels['xlabel'])
                plt.ylabel(labels['Quat_ylabels'][0])
                plt.subplot(412)
                plt.plot(self.t,self.q1)
                plt.xlabel(labels['xlabel'])
                plt.ylabel(labels['Quat_ylabels'][1])
                plt.subplot(413)
                plt.plot(self.t,self.q2)
                plt.xlabel(labels['xlabel'])
                plt.ylabel(labels['Quat_ylabels'][2])
                plt.subplot(414)
                plt.plot(self.t,self.q3)
                plt.xlabel(labels['xlabel'])
                plt.ylabel(labels['Quat_ylabels'][3])
        plt.show()

    def PlotFrequencyDomain(self):
        N = len(self.t)
        dt = self.t[-1]/N
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
        plt.subplot(311)
        plt.plot(xf,xdataf)
        plt.subplot(312)
        plt.plot(xf,ydataf)
        plt.subplot(313)
        plt.plot(xf,zdataf)
        plt.show()



def TrimData(data,trimTime):
    count = 0
    for ii in range(0,len(data)):
        if data[ii][7] >= trimTime:
            break
        else:
            count = count + 1
    print(count)
    return data[count:len(data)]

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
        try:
            b = ser.readline()
            if len(b) == BUFFER_LEN:
                if ii == 0:
                    start = time.time()
                    t = [0]
                else:
                    t = [time.time() - start]
                rawData.append(list(b) + list(t))
            ii = ii + 1
        except:
            print('Keyboard Interupt')
            break
    ser.close()
    return rawData

def ConvertRawData(rawData,gravity):
    data = []
    dataRow = [0,0,0,0,0,0,0,0]
    for ii in range(0,len(rawData)):
        b = rawData[ii]
        dataRow[0] = (b[2] << 24 | b[3] << 16 | b[4] << 8 | b[5])/1073741824.0
        dataRow[1] = (b[6] << 24 | b[7] << 16 | b[8] << 8 | b[9])/1073741824.0
        dataRow[2] = (b[10] << 24 | b[11] << 16 | b[12] << 8 | b[13])/1073741824.0
        dataRow[3] = (b[14] << 24 | b[15] << 16 | b[16] << 8 | b[17])/1073741824.0
        for jj in range(0,4):
            if dataRow[jj] >= 2:
                dataRow[jj] = dataRow[jj] - 4
        dataRow[4] = gravity*(b[18] << 24 | b[19] << 16 | b[20] << 8 | b[21])/536870912.0
        dataRow[5] = gravity*(b[22] << 24 | b[23] << 16 | b[24] << 8 | b[25])/536870912.0
        dataRow[6] = gravity*(b[26] << 24 | b[27] << 16 | b[28] << 8 | b[29])/536870912.0
        for jj in range(4,7):
            if dataRow[jj] >= gravity*4:
                dataRow[jj] = dataRow[jj] - gravity*8
        dataRow[7] = b[32]
        data.append(dataRow.copy())
    return data
