from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt

q0 = 0.5
a = 1

def func_chm(fi, t):
    dfi_dt = a * fi
    return dfi_dt

t = np.linspace(0, 10)
fi = odeint(func_chm, q0, t)

plt.figure()
plt.grid(True)
plt.xlabel('t', fontsize=12)
plt.ylabel('fi(t)', fontsize=12)
plt.plot(t, fi, color='g', label='chm')
plt.plot(t, q0 * np.exp(a * t), color='b', label='real', linestyle = ':')
plt.legend(loc='best')
plt.title('Сравнение явного и численного решений:')
plt.show()