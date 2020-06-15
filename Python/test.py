from math import *

v = [1,2,3,5]
sum = 0
vNew = []
for ii in range(0,len(v)):
	sum = sum + v[ii]**2

mag = sqrt(sum)
for ii in range(0,len(v)):
	vNew.append(v[ii]/mag)

for ii in range(0,len(vNew)):
	newSum = newSum + vNew[ii]**2
newMag = sqrt(newSum)

print(v)
print(vNew)
print(mag)
print(newMag)
