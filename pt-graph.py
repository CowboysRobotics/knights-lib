import matplotlib.pyplot as plt

x = []
y = []

f = open("test-file.txt", "r")

for line in f:
    try:
        if line[:3] == "ptg":
            arr = line.split(",")
            print(arr[1], arr[2])
            y.append(float(arr[1]))
            x.append(int(arr[2]))
    except:
        break

plt.plot(x,y)
plt.show()
