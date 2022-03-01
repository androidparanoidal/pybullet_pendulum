import pybullet as p
import time
import pybullet_data
from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

t = 0
dt = 1/240  # pybullet simulation step
q0 = 1.58   # starting position (radian) 1.58
physicsClient = p.connect(p.DIRECT)  # or p.DIRECT for non-graphical version or p.GUI
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
# planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("./simple.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
q0 = jointPosition

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

time_list = []
position_list = []

#while True:
#    p.stepSimulation()
#    time.sleep(dt)      или...

while t < 5:
    jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
    position_list.append(jointPosition)
    p.stepSimulation()

    time_list.append(t)
    t += dt
#   time.sleep(dt)

print(position_list)
print(time_list, '\n')

# Численное решение с помощью odeint
b = 1
m = 2
length = 0.8
g = 9.81
koef1 = g/length   # = 12.2625
koef2 = b/(m * length**2)  # = 1.5625
q0_initial = [q0, 0]  # нач знач: x(t=0) = 1, x'(t=0) = 0

# d^2x/dt^2 +  b/(m*l*l) * dx/dt + g/l * sinx = 0, где пусть
def odesol(X, t, b, m, length, g):
    x, x_new = X
    dx_new = [x_new, -(b/(m * length**2)) * x_new - (g/length) * np.sin(x)]
    return dx_new

t = np.linspace(0, 5, 5*240)
solution = odeint(odesol, q0_initial, t, args=(b, m, length, g))
xs = solution[:, 0]


# Численное решение по полунеявному м Эйлера (схема интегрирования в pybullet): v' = -koef*sin(u) и u'=v, замена в исходном v=x' и u=x
n = 1200-1 # кол-во шагов
h = 1/240 # шаг dt

u0 = q0
v0 = 0
u = np.zeros(n+1)
v = np.zeros(n+1)

t1 = np.zeros(n+1)
#t1 = np.linspace(0, n*h, n+1)

u[0] = u0
v[0] = v0

for i in range(n):
    t1[i+1] = t1[i] + h
    v[i+1] = v[i] - koef2 * h * v[i] - koef1 * h * np.sin(u[i])
    u[i+1] = u[i] + h * v[i+1]


# Оптимизация параметров д.у.
def odesol2(X, t, koef1, koef2):
    x, x_new = X
    dx_new = [x_new, - koef2 * x_new - koef1 * np.sin(x)]
    return dx_new

guess = [1.0, 11.0]  #initial_guess

def func(par):
    koef2, koef1 = par
    solution2 = odeint(odesol2, q0_initial, t, args=(koef1, koef2))
    norm = np.linalg.norm(solution2[:, 0] - solution[:, 0])
    #norm = np.mean(L2_norm_1(solution, solution2))
    return norm

res = minimize(func, np.array(guess), method='Nelder-Mead')
print(res, "\n")


def L2_norm_1(xs, position_list):
    distance1 = np.sqrt(abs((np.array(xs) - np.array(position_list)) ** 2))
    return distance1

def L2_norm_2(u, position_list):
    distance2 = np.sqrt(abs((np.array(u) - np.array(position_list)) ** 2))
    return distance2

'''
print('\nМодули разности между odeint и симуляторным решением: ', L2_norm_1(xs, position_list),
      '\nНорма L2 = ', np.mean(L2_norm_1(xs, position_list)), '\n')
print('\nМодули разности между м эйлера и симуляторным решением: ', L2_norm_2(u, position_list),
      '\nНорма L2 = ', np.mean(L2_norm_2(u, position_list)), '\n')
'''

plt.figure()
plt.grid(True)
plt.xlabel('t', fontsize=12)
plt.ylabel('x(t)', fontsize=12)
plt.plot(t, solution[:, 0], color='k', label='odeint', linestyle=':', linewidth=2)
plt.plot(t1, u, color='r', label='эйлер')
plt.plot(time_list, position_list, color='c', label='симуляторное')
plt.legend(loc='best')
plt.title('Сравнение симуляторного и численных решений:')
'''okr1 = round(np.mean(L2_norm_2(u, position_list)), 7)
okr2 = round(np.mean(L2_norm_2(xs, position_list)), 7)
plt.text(0, -0.8, "Норма разности симуляторного решения и")
plt.text(0, -0.9, "м. Эйлера = {}".format(okr1))
plt.text(0, -1, "odeint = {}".format(okr2)) '''
plt.show()

p.disconnect()
