import GolfProjectFunctions as GPF

BUFFER_LEN = 32
gravity = 9.81
rawData = GPF.getRawData('COM10', 115200, BUFFER_LEN)
data = GPF.convertRawData(rawData, gravity)
WF = GPF.Frame('World')
WF.fillData(data,gravity)
WF.getMotion()
WF.plotData()
