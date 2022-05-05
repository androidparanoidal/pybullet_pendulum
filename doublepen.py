import pybullet as p
import pybullet_data
from scipy.integrate import odeint
import numpy as np
import math

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
boxId = p.loadURDF("./double_pendulum.urdf", useFixedBase=True)

p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

dt = 1 / 240
pi = math.pi
dof = 2
joint_id = range(dof)
the0_1 = 0.1
the0_2 = 0.1
the_0 = [the0_1, the0_2]

tm = np.linspace(0, 5, 5*240)
time_list = []
positions_list = []
w_list = []

def double_pendulum_sim(the_0):
    t = 0
    # go to the starting position
    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetPositions=the_0, controlMode=p.POSITION_CONTROL)
    for _ in range(1000):
        p.stepSimulation()

    #joint_states = p.getJointStates(boxId, jointIndices=joint_id)
    #joint_positions = np.array([i[0] for i in joint_states])
    #the_0 = joint_positions

    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetVelocities=[0.0, 0.0], controlMode=p.VELOCITY_CONTROL, forces=[0.0, 0.0])
    positions_list = []

    while t < 5:
        joint_states = p.getJointStates(boxId, jointIndices=joint_id)
        joint_positions = np.array([i[0] for i in joint_states])
        joint_velocities = np.array([j[1] for j in joint_states])
        positions_list.append(joint_positions)
        w_list.append(joint_velocities)
        p.stepSimulation()
        time_list.append(t)
        t += dt

    #print(positions_list)
    return positions_list

sol_sim = double_pendulum_sim(the_0)
p.disconnect()


L1, L2 = 0.8, 0.7
M1, M2 = 2, 1.5
g = 9.81

i_c = [0.1, 0, 0.1, 0]  # initial conditions

T = int(5 / dt)
TM = [0] * T

def double_pendulum_mod(x, t, M1, M2, L1, L2, g):
    dx = np.zeros(4)

    c = np.cos(x[0] - x[2])
    s = np.sin(x[0] - x[2])

    dx[0] = x[1]
    dx[1] = (M2 * g * np.sin(x[2]) * c - M2 * s * (L1 * c * x[1] ** 2 + L2 * x[3] ** 2) - (M1 + M2) * g * np.sin(x[0])) / (L1 * (M1 + M2 * s ** 2))
    dx[2] = x[3]
    dx[3] = ((M1 + M2) * (L1 * x[1] ** 2 * s - g * np.sin(x[2]) + g * np.sin(x[0]) * c) + M2 * L2 * x[3] ** 2 * s * c) / (L2 * (M1 + M2 * s ** 2))

    return dx


sol = odeint(double_pendulum_mod, i_c, tm, args=(M1, M2, L1, L2, g))

x0 = sol[:, 0]
x1 = sol[:, 1]
x2 = sol[:, 2]
x3 = sol[:, 3]

