import serial
import time

# set up the serial line
ser = serial.Serial('/dev/cu.wchusbserial1410', 115200)
time.sleep(2)
ser.write(b'r')

# Read and record the data
data =[]                       # empty list to store the data
while True:
    b = ser.readline()         # read a byte string
    string_n = b.decode()  # decode byte string into Unicode
    string = string_n.rstrip() # remove \n and \r
    #print(string)
    flt = float(string)        # convert string to float
    print(flt)
    #data.append(flt)           # add to the end of data list
    #time.sleep(0.001)            # wait (sleep) 0.1 seconds

ser.close()

# show the data

for line in data:
    print(line)
