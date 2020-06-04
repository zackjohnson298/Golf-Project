class a():
	def __init__(self,Data):
        self.data = Data
return

b = a([0,1,2,3,4,5,6])

del b.data[0:2]

print(b.data)
