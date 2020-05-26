import serial
import time
import matplotlib.pyplot as plt

class SensorFrame():

    def __init__(self):
        self.q0 = []
        self.q1 = []
        self.q2 = []
        self.q3 = []
        self.xddot = []
        self.yddot = []
        self.zddot = []
        self.t = []
        self.pltLabels = {}
        self.pltLabels['xlabel'] = 'Time t [s]'
        self.pltLabels['Accel_ylabels'] = ['xddot [m/s^2]','yddot [m/s^2]','zddot [m/s^2]']
        self.pltLabels['Accel_title'] = 'Sensor Frame Accelerations'
        self.pltLabels['Quat_ylabels'] = ['q0','q1','q2','q3']
        self.pltLabels['Quat_title'] = 'Sensor Frame Quaternions'


    def fillData(self,data):
        for ii in range(0,len(data)):
            self.q0.append(data[ii][0])
            self.q1.append(data[ii][1])
            self.q2.append(data[ii][2])
            self.q3.append(data[ii][3])
            self.xddot.append(data[ii][4])
            self.yddot.append(data[ii][5])
            self.zddot.append(data[ii][6])
            self.t.append(data[ii][7])

def plotData(obj,values):
    for ii in range(0,len(values)):
        if values[ii] == 'acc':
            labels = obj.pltLabels
            plt.figure()
            plt.suptitle(labels['Accel_title'])
            plt.subplot(311)
            plt.plot(obj.t,obj.xddot)
            plt.xlabel(labels['xlabel'])
            plt.ylabel(labels['Accel_ylabels'][0])
            plt.subplot(312)
            plt.plot(obj.t,obj.yddot)
            plt.xlabel(labels['xlabel'])
            plt.ylabel(labels['Accel_ylabels'][1])
            plt.subplot(313)
            plt.plot(obj.t,obj.zddot)
            plt.xlabel(labels['xlabel'])
            plt.ylabel(labels['Accel_ylabels'][2])
        if values[ii] == 'quat':
            labels = obj.pltLabels
            plt.figure()
            plt.suptitle(labels['Quat_title'])
            plt.subplot(411)
            plt.plot(obj.t,obj.q0)
            plt.xlabel(labels['xlabel'])
            plt.ylabel(labels['Quat_ylabels'][0])
            plt.subplot(412)
            plt.plot(obj.t,obj.q1)
            plt.xlabel(labels['xlabel'])
            plt.ylabel(labels['Quat_ylabels'][1])
            plt.subplot(413)
            plt.plot(obj.t,obj.q2)
            plt.xlabel(labels['xlabel'])
            plt.ylabel(labels['Quat_ylabels'][2])
            plt.subplot(414)
            plt.plot(obj.t,obj.q3)
            plt.xlabel(labels['xlabel'])
            plt.ylabel(labels['Quat_ylabels'][3])
    plt.show()

def getRawData(port,baudrate,BUFFER_LEN):
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

def convertRawData(rawData,gravity):
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
