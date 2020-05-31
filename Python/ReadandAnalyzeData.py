import GolfProjectFunctions as GPF

BUFFER_LEN = 32
gravity = 9.81
rawData = GPF.GetRawData('COM10', 115200, BUFFER_LEN)
data = GPF.ConvertRawData(rawData, gravity)
WF = GPF.Frame('World')
WF.FillData(data, calibrate = False, calibrationTime = 5)
#WF.GetMotion()
WF.ButterFilter([6,6,6],6)
WF.PlotData(['acc'])
WF.PlotFrequencyDomain()
