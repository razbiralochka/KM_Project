from PID import pid_class
from numpy.linalg import norm
import math as m
import matplotlib.pyplot as plt
import numpy as np

rho = 1.29
Sm = m.pi * (1.5**2)/2
peng = 1200000
mass = 100000
w = 3000
dm = peng / w

def goal(time):
    res = m.pi/400*time
    return res

def aero_force(V,a):
    CY = 0.1*1000
    CX = 0.2
    wind = np.array([0, 0])
    if norm(V) > 0: a=a-wind[0]/norm(V)

    V = V+wind
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
    return np.matmul(vec, rot)


def atan(vec):
    if vec[1] == 0:
        res = 0

    else:
        res = m.atan(vec[0] / vec[1])

    return res

def engine_move(dang, beta):
    u_max = 0.1;
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


t = 0
h = 0.01




#pid = pid_class(h,0,10,0.1,4)
pid = pid_class(h,0,0.1,0,0)
while mass > 40000:
    t_list.append(t)
    x_list.append(Pos[0])
    y_list.append(Pos[1])
    vel_ang = atan(Vel)
    attack_angle = vel_ang - pitch



    R = aero_force(Vel, attack_angle)
    NQ = rot_2(R, attack_angle)

    goal_pitch = goal(t)
    goal_alt = 200000*np.exp(-0.00001*Pos[0])*(np.exp(0.00001*Pos[0])-1)

    pid.update_goal(goal_pitch)

    goal_phi = pid.gen_signal(pitch)
    if abs(phi > 4*m.pi/180):
        goal_phi = 4*m.pi/180*abs(phi)/phi


    beta = beta + engine_move(goal_phi-phi,beta)*h
    phi = phi + beta *h


    P = thrust(peng, phi)




    # минус-стабильно плюс-нестабильно
    omega = omega + (P[1] * 30 / 1e9 - NQ[1] * 15*attack_angle / 1e9) * h

    pitch = pitch + omega * h
    test_list.append(goal_pitch-pitch)
    Vel = Vel + (rot_1(R, vel_ang) + rot_1(P, pitch) + mass * g) * h / mass

    Pos = Pos + Vel * h
    mass = mass - dm * h
    t = t + h

def treck(x):

    y = 200000*np.exp(-0.00001*x)*(np.exp(0.00001*x)-1)
    return y

x_theor = np.linspace(0, 50000, num=10000)
plt.plot(x_list, y_list)
plt.plot(x_theor, treck(x_theor))
plt.axis('equal')
plt.show()
plt.plot(t_list, test_list)

plt.show()


