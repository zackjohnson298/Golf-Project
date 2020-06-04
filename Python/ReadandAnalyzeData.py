import GolfProjectFunctions as GPF

BUFFER_LEN = 32
gravity = 9.81
rawData = GPF.GetRawData('COM10', 115200, BUFFER_LEN)
fullData = GPF.ConvertRawData(rawData, gravity)
data = GPF.TrimData(fullData,1)
WF = GPF.Frame('World')
WF.FillData(data)
# WF.LowessFilter(.1)
# WF.ButterFilter([6,6,6],6)
WF.Calibrate(5)
# WF.GetMotion()
# WF.OffsetPosition(2,5,10)
WF.HardcodeBS(0.1)
WF.GetMotion()
WF.PlotData(['acc','pos'])
# WF.PlotFrequencyDomain()
