import pybullet as p
import time
import pybullet_data
from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt

t = 0
dt = 1/240  # pybullet simulation step
q0 = 0.5   # starting position (radian)
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
#planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("./simple.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

time_list = []
position_list = []

#while True:
#    p.stepSimulation()
#    time.sleep(dt)      или...

while t < 5:
    p.stepSimulation()
    time.sleep(dt)

    time_list.append(t)
    t += dt

    jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
    position_list.append(jointPosition)


# Численное решение с помощью odeint
length = 0.8
g = 9.81
koef = g/length
q0_initial = [0.5, 0]  # нач знач: x(t=0) = 0.5, x'(t=0) = 0

# d^x/dt^2 + g/l * x = 0 решение которого: x(t) = c1*sin(sqrt(koef)*t) + c2*cos(sqrt(koef)*t)
def diff_pend(x, t):
    return [x[1], -koef * x[0]]

t = np.linspace(0, 5, 5*240)
solution = odeint(diff_pend, q0_initial, t)
xs = solution[:, 0]


# Численное решение по полунеявному м Эйлера (схема интегрирования в pybullet): v' = -koef*u и u'=v, замена в исходном v=x' и u=x
u0 = 0.5
v0 = 0
n = 1200 # кол-во шагов
h = 1/240 # шаг dt
u = np.zeros(n+1)
v = np.zeros(n+1)
vremya = np.linspace(0, n*h, n+1)
u[0] = u0
v[0] = v0

for i in range(n):
    v[i+1] = v[i] - koef * h * u[i]
    u[i+1] = u[i] + h * v[i+1]


def L2_norm(xs, position_list):
    distance = np.sqrt(abs((np.array(xs) - np.array(position_list))**2))
    return distance

print('\nМодули разности: ', L2_norm(xs, position_list),
      '\nНорма L2 = ', np.mean(L2_norm(xs, position_list)), '\n')

plt.figure()
plt.grid(True)
plt.xlabel('t', fontsize=12)
plt.ylabel('x(t)', fontsize=12)
plt.plot(time_list, position_list, color='c', label='симуляторное')
plt.plot(t, xs, color='k', label='численное', linestyle = ':')
plt.plot(vremya, u, color='r', label='эйлер')
plt.legend(loc='best')
plt.title('Сравнение симуляторного и численного решений:')
#plt.text(0, -0.4, "L2 Норма = ПОНЯТЬ КАК ДОБАВИТЬ ПЕРЕМЕННУЮ....... ",)
plt.show()

p.disconnect()