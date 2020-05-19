import serial
ser = serial.Serial('COM8')
ser.flushInput()

while True:
    try:
        ser_bytes = ser.readLine()
        decoded_bytes = float(ser_bytes():len(ser_bytes)-2).decode("utf-8")
        print(decoded_bytes)
    except:
        print("Keyboard Interrupt")
        break
