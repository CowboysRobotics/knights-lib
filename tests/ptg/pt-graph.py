import matplotlib.pyplot as plt

x = []
y = []

f = open("test-file.txt", "r")

for line in f:
    if line[:3] == "ptg":
        arr = line.split(",")
        print(arr[1], arr[2])
        x.append(int(arr[1]))
        y.append(int(arr[2]))

plt.plot(x,y)
plt.show()
