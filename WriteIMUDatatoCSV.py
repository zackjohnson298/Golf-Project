import serial
import time
import csv

# set up the serial line
ser = serial.Serial('COM10', 115200)
time.sleep(2)
ser.write(b'r')

# Read and record the data
data = [0,0,0,0,0,0,0,0]                       # empty list to store the data

for i in range(0,16):
    b = ser.readline()         # read a byte string

while True:
    try:
        b = ser.readline()
        #print(len(b))
        if len(b) == 18:
            data[0] = (b[2] << 8 | b[3])/16384.0
            data[1] = (b[4] << 8 | b[5])/16384.0
            data[2] = (b[6] << 8 | b[7])/16384.0
            data[3] = (b[8] << 8 | b[9])/16384.0
            for i in range(0,4):
                if data[i] >= 2:
                    data[i] = data[i] - 4
            data[4] = (b[10] << 8 | b[11])/8192.0
            data[5] = (b[12] << 8 | b[13])/8192.0
            data[6] = (b[14] << 8 | b[15])/8192.0
            for i in range(4,7):
                if data[i] >= 4:
                    data[i] = data[i] - 8
            data[7] = time.time()
            # print(data[4:7])
            with open("test_data.csv","a") as f:
                writer = csv.writer(f,delimiter=",")
                writer.writerow(data)
    except:
        print('Keyboard Interupt')
        break
ser.close()

# show the data

for line in data:
    print(line)
