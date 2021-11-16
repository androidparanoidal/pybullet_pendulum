import numpy as np
import matplotlib.pyplot as plt

with open("test.txt", "r") as tf:
    lines = tf.read().split(',')

list = []

for i in lines:
    list.append(int(i))
    #print(i)

print(list)
len = np.size(list)
print("\nРазмерность = ", len, "\n")

t = np.linspace(0, len, len)

plt.figure()
plt.grid(True)
plt.xlabel('t', fontsize=12)
plt.ylabel('Показания', fontsize=12)
plt.plot(t, list, color='b', marker = 'o', label='dsfsjdbf')
plt.legend(loc='best')
plt.title('График показаний потенциометра:')
plt.show()