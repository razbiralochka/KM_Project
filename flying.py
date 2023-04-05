from PID import pid_class
from numpy.linalg import norm
import math as m
import matplotlib.pyplot as plt
import numpy as np



peng = 395900
peng2 = 58800

w = 3700
dm = peng / w
s1 = 11
s2 = 9
mb1 = 27640
mb2 = 5000

#Плотности
po = 1140
pg = 440
kompsot = 3.5

#Основные габариты
D = 1.7
Lgo = 3.4+0.1
Loh1 = 2.8
Loh2 = 2.1
Lst1 = 17.4
Lst2 = 4.7
Lrocket = 25.6
#Массы компонентов топлива (константы)
mt1 = 25120
mt2 = 4440
#Массы компонентов топлива
mg1 = mt1 * 1 / (1 + kompsot)
mg2 = mt2 * 1 / (1 + kompsot)
mo1 = mt1 * kompsot / (1 + kompsot)
mo2 = mt2 * kompsot / (1 + kompsot)

#Объемы компонентов топлива
wg1 = mg1 / pg
wg2 = mg2 / pg
wo1 = mo1 / po
wo2 = mo2 / po

#Массы и объемы конструкции блоков
mk1 = mb1 - mt1
mk2 = mb2 - mt2
wk1 = mk1/2700
wk2 = mk2/2700

mass = mk1 + mk2 + mt1 + mt2 + 1000

#Высоты столба жидкости
Lg1 = wg1 * 4 / (np.pi*(D**2))
Lg2 = wg2 * 4 / (np.pi*(D**2))
Lo1 = wo1 * 4 / (np.pi*(D**2))
Lo2 = wo2 * 4 / (np.pi*(D**2))

#Высоты баков
Ug1 = Lg1*1/0.9
Ug2 = Lg2*1/0.9
Uo1 = Lo1*1/0.9
Uo2 = Lo2*1/0.9

#Уровни топлива
K21o = Lgo + Uo2 - Lo2
K22o = Lgo + Uo2
K2summo = K21o + K22o

K21g = K22o + Ug2 - Lg2
K22g = K22o + Ug2
K2summg = K21g + K22g

K11o = K22g + Loh2 + Uo1 - Lo1
K12o = K22g + Loh2 + Uo1
K1summo = K11o + K12o

K11g = K12o + Uo1 - Lg1
K12g = K12o + Uo1
K1summg = K11g + K12g

#Стартовые моменты инерции:

#_Конструкция
K2summ = K21o + K22g
K1summ = K11o + K12g
Sk2 = 0.5 * K2summ * mk2
Sk1 = 0.5 * K1summ * mk1
Sk = Sk1 + Sk2

Ik2 = 0.25 * ((K2summ**2) + (D/2)**2 + 0.333 * (Lst2**2)) * mk2
Ik1 = 0.25 * ((K1summ**2) + (D/2)**2 + 0.333 * (Lst1**2)) * mk1

Ik = Ik1 + Ik2
#_Топливо
Stg1 = mg1 * K1summg * 0.5
Stg2 = mg2 * K2summg * 0.5

Sto1 = mo1 * K1summo * 0.5
Sto2 = mo2 * K2summo * 0.5

Sto = Sto1 + Sto2
Stg = Stg1 + Stg2

Itg1 = mg1 * ((K1summg**2) + (D/2)**2 + 0.333 * (Ug1**2)) * 0.25
Itg2 = mg2 * ((K2summg**2) + (D/2)**2 + 0.333 * (Ug2**2)) * 0.25

Ito1 = mo1 * ((K1summo**2) + (D/2)**2 + 0.333 * (Uo1**2)) * 0.25
Ito2 = mo2 * ((K2summo**2) + (D/2)**2 + 0.333 * (Uo2**2)) * 0.25


Ito = Ito1 + Ito2
Itg = Itg1 + Itg2
Ssumm = Sto + Stg + Sk + (D * (Lgo**2) / 24)
Xcm = Ssumm / mass
Isumm = Ito + Itg + Ik + 127 - mass*(Xcm**2)


print(Xcm)
print(Lst1+Lst2+Lgo)


def goal(time):

    if time<2:
        res = 0
    else:
        res = m.pi/2+m.atan(1-300/t)


    #res = (m.pi / 2) * m.exp(-0.006 * time) * (m.exp(0.006 * time) - 1)
    return res

def aero_force(V,a,h):
    CY = 0.6*25*3
    CX = 0.12*m.pi*(1.6/2)**2


    w = 100*m.sin(0.05*h)

    wind = np.array([-w, 0])


    a=a-m.atan(wind[0]/(norm(V)+0.0001))

    rho = 1.2*m.exp(-h/8)
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
    res = m.atan2(vec[0], vec[1])

    return res

def engine_move(dang, beta):
    u_max = 0.2;
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
xc_list = list()
inert_list = list()
stable_list = list()
v_list = list()


nx_list=list()
ny_list=list()
Vel = np.array([0, 0])
Pos = np.array([0, 0])
omega = 0
pitch = 0
phi = 0
beta = 0
g = np.array([0, -9.81])


t = 0
h = 0.01




pid = pid_class(h,0,1,0.1,1)

while mt1 > 0:
    if Pos[1] < 0:
        print('Falling')
        break
    t_list.append(t)
    x_list.append(Pos[0]/1000)
    y_list.append(Pos[1]/1000)
    vel_ang = atan(Vel)
    attack_angle = vel_ang - pitch



    attack_angle, R = aero_force(Vel, attack_angle,Pos[1]/1000)


    test_list.append(180 * vel_ang / m.pi)
    test_list2.append(180 * pitch / m.pi)
    test_list3.append(180 * attack_angle / m.pi)


    NQ = rot_2(R, np.pi/2-attack_angle)

    goal_pitch = goal(t)




    pid.update_goal(goal_pitch)

    goal_phi = pid.gen_signal(pitch)
    if abs(goal_phi) > 20*m.pi/180:
        goal_phi = 20*m.pi/180*np.sign(goal_phi)


    beta = beta + engine_move(goal_phi-phi, beta)*h

    phi = phi + beta *h


    P = thrust(peng, phi)

    mt1 = mt1 - dm * h

    # Массы компонентов топлива
    mg1 = mt1 * 1 / (1 + kompsot)
    mg2 = mt2 * 1 / (1 + kompsot)
    mo1 = mt1 * kompsot / (1 + kompsot)
    mo2 = mt2 * kompsot / (1 + kompsot)

    # Объемы компонентов топлива
    wg1 = mg1 / pg
    wg2 = mg2 / pg
    wo1 = mo1 / po
    wo2 = mo2 / po


    mass = mk1 + mk2 + mt1 + mt2 + 1000

    # Длины баков
    Lg1 = wg1 * 4 / (np.pi * (D ** 2))
    Lg2 = wg2 * 4 / (np.pi * (D ** 2))

    Lo1 = wo1 * 4 / (np.pi * (D ** 2))
    Lo2 = wo2 * 4 / (np.pi * (D ** 2))


    # Уровни топлива
    K21o = Lgo + Uo2 - Lo2
    K22o = Lgo + Uo2
    K2summo = K21o + K22o

    K21g = K22o + Ug2 - Lg2
    K22g = K22o + Ug2
    K2summg = K21g + K22g

    K11o = K22g + Loh2 + Uo1 - Lo1
    K12o = K22g + Loh2 + Uo1
    K1summo = K11o + K12o

    K11g = K12o + Uo1 - Lg1
    K12g = K12o + Uo1
    K1summg = K11g + K12g

    # Стартовые моменты инерции:

    # _Конструкция
    K2summ = Lgo + K22g
    K1summ = Lgo + K22g + K12g




    # _Топливо
    Stg1 = mg1 * K1summg * 0.5
    Stg2 = mg2 * K2summg * 0.5

    Sto1 = mo1 * K1summo * 0.5
    Sto2 = mo2 * K2summo * 0.5

    Sto = Sto1 + Sto2
    Stg = Stg1 + Stg2

    Ssumm = Sto + Stg + Sk + (D * (Lgo ** 2) / 24)

    Itg1 = mg1 * ((K1summg ** 2) + (D / 2) ** 2 + 0.333 * (Ug1 ** 2)) * 0.25
    Itg2 = mg2 * ((K2summg ** 2) + (D / 2) ** 2 + 0.333 * (Ug2 ** 2)) * 0.25

    Ito1 = mo1 * ((K1summo ** 2) + (D / 2) ** 2 + 0.333 * (Uo1 ** 2)) * 0.25
    Ito2 = mo2 * ((K2summo ** 2) + (D / 2) ** 2 + 0.333 * (Uo2 ** 2)) * 0.25

    Ito = Ito1 + Ito2
    Itg = Itg1 + Itg2

    Xcm = Ssumm / mass
    Isumm = Ito + Itg + Ik + 127 - mass*(Xcm**2)


    # минус-стабильно плюс-нестабильно
    omega = omega + (P[1] * (26-Xcm) / Isumm + NQ[1] * (14-Xcm) / Isumm) * h

    stable_list.append(100*(14-Xcm)/26)

    nx_list.append((P[0]-NQ[0])/(mass*9.81))
    ny_list.append((P[1]+NQ[1])/(mass*9.81))

    pitch = pitch + omega * h

    magic = mass * np.array([0, 1])*(Vel[0]**2)/(6371000+Pos[1])

    Vel = Vel + (rot_1(R, vel_ang) + rot_1(P, pitch) + mass * g+magic) * h / mass

    Pos = Pos + Vel * h

    inert_list.append(Isumm)
    xc_list.append(Xcm)
    test_list4.append(NQ[1]/1000)
    test_list5.append(NQ[0]/1000)
    test_list6.append(omega*180/np.pi)
    v_list.append(norm(Vel))


    t = t + h


print('Cкорости(км/c): ',norm(Vel)/1000)
print('Высота(км): ',Pos[1] /1000)



fig, ax1 = plt.subplots()

color = 'tab:red'
ax1.set_xlabel('Время полёта, с')
ax1.set_ylabel('Момент  инерции, кг*м^2 ', color=color)
ax1.plot(t_list, inert_list, color=color)
ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

color = 'tab:blue'
ax2.set_ylabel('Координта центра тяжести, м', color=color)  # we already handled the x-label with ax1
ax2.plot(t_list, xc_list, color=color)
ax2.tick_params(axis='y', labelcolor=color)


fig.tight_layout()  # otherwise the right y-label is slightly clipped
plt.grid()
plt.show()





plt.plot(x_list, y_list, label='Траектория ')

plt.xlabel('Дальность полета, км')
plt.ylabel('Высота полета, км')
plt.grid()
plt.show()

plt.plot(t_list, v_list, label='Скорость по времени')

plt.xlabel('Время полёта, с')
plt.ylabel('Скорость, м/c')
plt.grid()
plt.show()


plt.plot(t_list, test_list, label = 'Угол наклона траектории')
plt.plot(t_list, test_list2,  label = 'Угол тангажа')
plt.plot(t_list, test_list3, label = 'Угол Атаки')
plt.legend()
plt.xlabel('Время полёта, с')
plt.ylabel('Градус')
plt.grid()
plt.show()


fig, ax1 = plt.subplots()

color = 'tab:red'
ax1.set_xlabel('Время полёта, с')
ax1.set_ylabel('Продольная сила, кН ', color=color)
ax1.plot(t_list, test_list5, color=color)
ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

color = 'tab:blue'
ax2.set_ylabel('Поперечная сила, кН', color=color)  # we already handled the x-label with ax1
ax2.plot(t_list, test_list4, color=color)
ax2.tick_params(axis='y', labelcolor=color)

fig.tight_layout()  # otherwise the right y-label is slightly clipped

ax1.set_ylim([-80,80])
ax2.set_ylim([-4,4])

plt.grid()
plt.show()




plt.plot(t_list[750:-1], test_list6[750:-1], label='вращение по тангажу ')

plt.xlabel('Время полёта, с')
plt.ylabel('град/c')
plt.grid()
plt.show()

plt.plot(t_list, stable_list, label='Показатель устойчивости ')

plt.xlabel('Время полёта, с')
plt.ylabel('Показатель устойчивости %')
plt.grid()
plt.show()


fig, ax1 = plt.subplots()

color = 'tab:red'
ax1.set_xlabel('Время полёта, с')
ax1.set_ylabel('Продольная перегрузка ', color=color)
ax1.plot(t_list, nx_list, color=color)
ax1.tick_params(axis='y', labelcolor=color)
ax1.set_ylim([-1,6])

ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

color = 'tab:blue'
ax2.set_ylabel('Поперечная перегрузка', color=color)  # we already handled the x-label with ax1
ax2.plot(t_list[750:-1], ny_list[750:-1], color=color)
ax2.tick_params(axis='y', labelcolor=color)

fig.tight_layout()  # otherwise the right y-label is slightly clipped
plt.grid()
plt.show()