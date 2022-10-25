from PID import pid_class
from numpy.linalg import norm
import math as m
import matplotlib.pyplot as plt
import numpy as np



peng = 8100000
mass = 549000
w = 2910
dm = peng / w


def goal(time):
    res = (m.pi/2)*m.exp(-0.005*time)*(m.exp(0.005*time)-1)
    return res

def aero_force(V,a,h):
    CY = 0.6*70*3
    CX = 0.08*m.pi*(3.7/2)**2
    w = 0
    if 0 < h <= 10.5:
        w = 11.5 * m.exp(0.195 * h)
    if 10.5 < h <= 27:
        w = 21 * m.exp(4.93 * (1e-3) * (27 - h) ** 2)
    if h > 27:
        w = 21

    wind = np.array([-0, 0])

    if norm(V) < 0.5: a=a - m.pi/2
    else: a=a-m.atan(wind[0]/norm(V))

    rho = 1.29*m.exp(-h/8)
    V = V+wind
    mas = np.array([-CX * m.cos(a),
                     CY * m.sin(a)])

    mas = mas * rho * (norm(V) ** 2) / 2

    return a, mas


def thrust(P_, ang):
    mas = np.array([m.cos(ang),
                    -m.sin(ang)])
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
    u_max = 0.3;
    u = u_max * np.sign(dang - beta*abs(beta)/(2*u_max))

    return u
x_list = list()
y_list = list()
t_list = list()
test_list = list()
test_list2 = list()
test_list3 = list()
test_list4 = list()
test_list5 = list()
test_list6 = list()
test_list7 = list()
Vel = np.array([0, 0])
Pos = np.array([0, 0])
omega = 0
pitch = 0
phi =0
beta = 0
g = np.array([0, -9.81])


t = 0
h = 0.01




pid = pid_class(h,0,1,0.1,2)

while mass > 140000:
    if Pos[1] < 0:
        print('Falling')
        break
    t_list.append(t)
    x_list.append(Pos[0])
    y_list.append(Pos[1])
    vel_ang = atan(Vel)
    attack_angle = vel_ang - pitch



    attack_angle, R = aero_force(Vel, attack_angle,Pos[1]/1000)

    test_list.append(180 * vel_ang / m.pi)
    test_list2.append(180 * pitch / m.pi)
    test_list3.append(180 * attack_angle / m.pi)


    NQ = rot_2(R, attack_angle)

    goal_pitch = goal(t)




    pid.update_goal(goal_pitch)

    goal_phi = pid.gen_signal(pitch)
    if abs(goal_phi) > 10*m.pi/180:
        goal_phi = 10*m.pi/180*np.sign(goal_phi)


    beta = beta + engine_move(goal_phi-phi, beta)*h
    phi = phi + beta *h


    P = thrust(peng, phi)

    inertia = mass * (70 ** 2) / 12

    # минус-стабильно плюс-нестабильно
    omega = omega + (P[1] * (35) / inertia + NQ[1] * (10)/ inertia) * h

    pitch = pitch + omega * h

    magic =mass * np.array([0, 1])*(Vel[0]**2)/(6371000+Pos[1])

    Vel = Vel + (rot_1(R, vel_ang) + rot_1(P, pitch) + mass * g+magic) * h / mass

    Pos = Pos + Vel * h
    mass = mass - dm * h

    test_list4.append(R[1])
    test_list5.append(rot_1(R, vel_ang)[1])
    test_list6.append(omega)

    t = t + h
print('Cкорости(км/ч): ',norm(Vel) * 3.6)
print('Высота(км): ',Pos[1] /1000)
plt.plot(x_list, y_list)

plt.show()
plt.plot(t_list, test_list, label = 'Наклон траектории')
plt.plot(t_list, test_list2,  label = 'Тангаж')
plt.plot(t_list, test_list3, label = 'Угол Атаки')
plt.legend(loc=4)
plt.xlabel('Секунда полёта')
plt.ylabel('Градус')
plt.grid()
plt.show()

plt.plot(test_list4, t_list,label='подъёмная сила в скоростной системе')
plt.plot(test_list5, t_list,label='подъёмная сила в земной системе')
plt.legend(loc=4)
plt.ylabel('Секунда полёта')
plt.xlabel('Ньютон')
plt.grid()
plt.show()

plt.plot(t_list, test_list6, label='вращение по тангажу ')

plt.legend(loc=4)
plt.xlabel('Секунда полёта')
plt.ylabel('рад/c')
plt.grid()
plt.show()