from PID import pid_class
from numpy.linalg import norm
import math as m
import matplotlib.pyplot as plt
import numpy as np

rho = 1.29
Sm = m.pi * m.pow(2.05, 2) / 2
peng = 1300000
mass = 100000
w = 3000
dm = peng / w

def goal(time):
    res = m.pi/400*time
    return res

def aero_force(V, a):
    CY = 0.05
    CX = 0.1
    mas = np.array([-CX, CY * a])

    mas = mas * rho * Sm * (norm(V) ** 2) / 2

    return mas


def thrust(P_, ang):
    mas = np.array([m.cos(-ang),
                    m.sin(-ang)])
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
    if vec[1] == 0:
        res = 0

    else:
        res = m.atan(vec[0] / vec[1])

    return res

def engine_move(dang, beta):
    u_max = 2*m.pi;
    u = u_max * np.sign(dang - beta*abs(beta)/(2*u_max))

    return u
x_list = list()
y_list = list()
t_list = list()
test_list = list()
test_list2 = list()
Vel = np.array([0, 0])
Pos = np.array([0, 0])
omega = 0
pitch = 0
phi =0
beta = 0
g = np.array([0, -9.81])
wind = np.array([10, 0])

t = 0
h = 0.01




pid = pid_class(h,0,10,0.1,4)

while mass > 50000:
    t_list.append(t)
    x_list.append(Pos[0])
    y_list.append(Pos[1])
    vel_ang = atan(Vel - wind)
    attack_angle = vel_ang - pitch



    R = aero_force(Vel-wind, attack_angle)
    NQ = rot_2(R, attack_angle)

    goal_pitch = goal(t)

    pid.update_goal(goal_pitch)



    goal_phi = pid.gen_signal(pitch)
    if abs(phi > m.pi/180):
        goal_phi = m.pi/180*abs(phi)/phi


    beta = beta + engine_move(goal_phi-phi,beta)*h
    phi = phi + beta *h


    test_list.append(180 * (phi) / m.pi)


    P = thrust(peng, phi)
    # минус-стабильно плюс-нестабильно
    omega = omega + (P[1] * 10 / 1e7 - NQ[1] * 20 / 1e7) * h

    pitch = pitch + omega * h

    Vel = Vel + (rot_1(R, vel_ang) + rot_1(P, pitch) + mass * g) * h / mass

    Pos = Pos + Vel * h
    mass = mass - dm * h
    t = t + h

plt.plot(x_list, y_list)
plt.axis('equal')
plt.show()
plt.plot(t_list, test_list)

plt.show()


