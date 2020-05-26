import GolfProjectFunctions as GPF

BUFFER_LEN = 32
gravity = 9.79
rawData = GPF.getRawData('COM10', 115200, BUFFER_LEN)
data = GPF.convertRawData(rawData, gravity)
SF = GPF.SensorFrame()
SF.fillData(data)
GPF.plotData(SF,['acc','quat'])
