import numpy as np
import matplotlib.pyplot as plt
import pylab
import math
import volts2

with open("volts.txt", "r") as tf:
    lines = tf.read().split(',')

u = []  # напряжение
for i in lines:
    u.append(int(i))
    #print(i)

print(u)
len = np.size(u)
#print("\nРазмерность = ", len, "\n")
t = np.linspace(0, len, len)
n = np.size(u)
t2 = np.zeros(n)  # не работает как надо...
dt = 1/40


a = 906
b = 819
c = 0
d = (math.pi)/2

u2 = volts2.vl
alpha = []

t3 = np.linspace(0, int(817/40), 817)

for k in range(np.size(u2)):  #или for k in u2:
    #t2[k] = t2[k - 1] + dt
    j = (((u2[k] - a) * (d - c)) / (b - a)) + c
    #j = d / a * (-u2[k] + a)
    alpha.append(j)
    #print(j)

print('ugly: ', alpha)
#print(t2, '\n')
#print(np.size(alpha))
#print(np.size(t2))


'''
От a до b - напряжение от 908 до 0 в усл. ед.
c и d - углы от 0 до pi/2 соотв-но
f(x) = [ ((x-a) * (d-c)) / (b-a) ] + c
'''

def f(x):
    return d * (-x + a) / a

# отсортированный по возрастанию массив уголов (не особо нужен.)
fi = []
for x in np.linspace(a, b, a):
    alpha_2 = f(x)
    #print("({}, {})".format(x, alpha))
    fi.append(alpha_2)
#print(fi)


'''
plt.figure()
plt.grid(True)
plt.xlabel('t', fontsize=12)
plt.ylabel('Напряжение в усл. ед.', fontsize=12)
plt.plot(t, u, color='b', marker='o', label='Показания')
plt.legend(loc='best')
plt.title('График показаний потенциометра:')
plt.show()
'''


pylab.figure(1)
pylab.grid()
pylab.title("График показаний потенциометра:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('Напряжение в усл. ед.', fontsize=12)
pylab.plot(t, u, color='b', marker='o', label='Показания')
pylab.legend()

pylab.figure(2)
pylab.grid()
pylab.xlabel('t', fontsize=12)
pylab.ylabel('alpha', fontsize=12)
pylab.title("Изменение угла со временем:")
pylab.plot(t3, alpha, color='r', label = " ")
pylab.legend()

pylab.show()