import math

file = open("data.txt", 'w')

for i in range(10000):
    a = 1.0 - 1 / (1.0 + i * 0.01)
    file.write(str(a) + " " + str(1.0) + " " + str(1.0 - a) + '\n')

file.close()
