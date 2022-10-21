import matplotlib.pyplot as plt
import numpy as np
from PID import pid_class
from numpy.linalg import norm
import math as m

rho = 1.29
Sm = m.pi * m.pow(2.05, 2) / 2
peng = 1765800
mass = 100000
w = 3000
dm = peng / w




def aero_force(V, a):
    CY = 0.05
    CX = 0.1
    mas = np.array([-CX, CY * a])

    mas = mas * rho * Sm * (norm(V) ** 2) / 2

    return mas


def thrust(P_, ang):
    mas = np.array([m.sin(ang),
                    m.cos(ang)])
    return P_ * mas


def rot_1(vec, ang):
    rot = np.array([[np.sin(ang), np.cos(ang)],
                    [-np.cos(ang), np.sin(ang)]])

    return np.matmul(vec, rot)


def rot_2(vec, ang):
    rot = np.array([[np.cos(ang), np.sin(ang)],
                    [-np.sin(ang), np.cos(ang)]])
    return np.matmul(vec+wind, rot)


def atan(vec):
    if norm(vec) < 20:
        res = 0
        print(res)
    else:
        res = m.atan(vec[0] / vec[1])

    return res


x_list = list()
y_list = list()
t_list = list()
test_list = list()

Vel = np.array([0, 0])
Pos = np.array([0, 0])
omega = 0
pitch = 0
g = np.array([0, -9.81])
wind = np.array([5, 0])

t = 0
h = 0.01
pid = pid_class(h,0,10,0,0)
while mass > 70000:
    t_list.append(t)
    x_list.append(Pos[0])
    y_list.append(Pos[1])
    vel_ang = atan(Vel - wind)
    attack_angle = vel_ang - pitch

    test_list.append(pitch*180/m.pi)

    R = aero_force(Vel-wind, attack_angle)
    P = np.array([peng, 0])

    NQ = rot_2(R, attack_angle)



    omega = omega + (P[1] * 10 / 1e6 + NQ[1] * 20 / 1e6) * h

    pitch = pitch + omega * h

    Vel = Vel + (rot_1(R, vel_ang) + rot_1(P, pitch) + mass * g) * h / mass

    Pos = Pos + Vel * h
    mass = mass - dm * h
    t = t + h

plt.plot(x_list, y_list)
plt.show()
plt.plot(y_list, test_list)
plt.show()


