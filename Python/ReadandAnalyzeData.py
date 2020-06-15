import GolfProjectFunctions as GPF

BUFFER_LEN = 44
GYRO_FS = 250
gravity = 1
rawData = GPF.GetRawData('COM10', 115200, BUFFER_LEN)
fullData = GPF.ConvertRawData(rawData, gravity, GYRO_FS)
data = GPF.TrimData(fullData,1)
WF = GPF.Frame('World')
WF.FillData(data)
# WF.LowessFilter(.1)
# WF.ButterFilter([6,6,6],6)
# WF.Calibrate(5)
# WF.GetMotion()
# WF.OffsetPosition(2,5,10)
# WF.HardcodeBS(0.1)
WF.GetVelZUPT(thresh = .035, lowCutoff = 10, highCutoff = .001)
WF.GetClubheadSpeed(1)
# WF.GetPosition()
# WF.PlotFrequencyDomain()
WF.PlotData(['Gyro','Club'])
