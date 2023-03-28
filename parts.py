import numpy as np

class Fuel_tank:
    def __init__(self, dia, lenght, constr_mass , liquid_type = "LNG"):
        self._diameter  = dia
        self._radius = dia / 2
        self._lenght = lenght
        self._constr_mass = constr_mass

        if liquid_type == "LNG":
            self._liquid_density = 440
        if liquid_type == "LOX":
            self._liquid_density = 1140
        if liquid_type == "Kerosine":
            self._liquid_density = 840
        if liquid_type == "Hydrogene":
            self._liquid_density = 70

        self._liquid_lenght = 0.9*lenght
        self._liquid_volume = self._liquid_lenght*np.pi*(dia**2)/4
        self._liquid_mass = self._liquid_density*self._liquid_volume
        self._total_mass = self._constr_mass + self._liquid_mass
        self._constr_inertia = self._constr_mass * ((self._radius ** 2) / 2 + (self._lenght ** 2) / 3)
    def burnout(self, consumption, time):
        pass
    def part_inertia(self):
        offset = self._lenght-self._liquid_lenght/2

        liquid_inertia = self._liquid_mass*((self._radius**2)/4+(self._liquid_lenght**2)/12+offset**2)

        total_inetria = self._constr_inertia+liquid_inertia

        return total_inetria

    def part_mass(self):

        return self._total_mass

    def part_lenght(self):
        return self._lenght
    def CoM(self):
        offset = self._lenght - self._liquid_lenght / 2
        res = self._constr_mass*self._lenght/2 + self._liquid_mass*(self._lenght-offset)
        return res/self._total_mass




class Engine:
    def __init__(self, imp, cunsuption):
        self._imp = imp
        self._cunsuption = cunsuption
        self._thrust = imp*cunsuption
        print("Тяга кН ", self._thrust/1000)
        a = 10959.9
        b = 0.000165262
        self._mass = self._thrust*(35/25000)
        print("Масса ДУ ", self._mass)
    def part_mass(self):

        return self._mass
    def part_inertia(self):
        return 0

    def part_lenght(self):
        return 0

    def CoM(self):
        return 0
class Adapter:
    def __init__(self, upper_dim, lower_dim, lenght, mass):
        self._upper_dim = upper_dim
        self._lower_dim = lower_dim
        self._lenght = lenght
        r1 = upper_dim/2
        r2 = lower_dim/2
        self._inertia = (lenght**2)*mass/(r1+3*r2)/(6*(r1+r2))+0.25*mass*(pow(r2, 2)+pow(r1, 2))
        self._CoM = self._lenght/3+(2*self._lower_dim+self._upper_dim)/(self._lower_dim+self._upper_dim)
        self._mass = mass
    def part_inertia(self):
        return self._inertia

    def part_lenght(self):
        return self._lenght

    def part_mass(self):
        return self._mass
    def CoM(self):
        return self._CoM
class Fairing:
    def __init__(self, lenght, dim, mass):
        self._lenght = lenght
        self._dim = dim
        self._radius = dim/2
        self._mass = mass
        self._static = 0
        self._density = 0
        self.upd_masses()

    def radius(self,arg):
        res =0.5*((self._radius**2-self._lenght**2)/self._radius+np.sqrt(pow(self._lenght,4)/pow(self._radius,2)
                                                                    +pow(self._radius,2)-2*pow(self._lenght,2)+
                                                                    8*self._lenght*arg-4*pow(arg,2)))
        return res

    def inert(self,arg):
        res = (pow(self.radius(arg), 2) / 2 + pow(arg, 2)) * self._density * self.radius(arg)

        return res
    def upd_masses(self):
        l = np.linspace(0, self._lenght, 10000)
        surface = 2 * np.pi * np.trapz(self.radius(l), l)
        self._density = self._mass / surface
        self._inertia = np.trapz(self.inert(l), l)
        self._static = self._density * 2 * np.pi * np.trapz(self.radius(l) * l, l) / self._mass

    def part_inertia(self):
        return self._inertia

    def part_lenght(self):
        return self._lenght

    def part_mass(self):
        return self._mass
    def CoM(self):
        return  self._static

class Payload:
    def __init__(self, mass):
        self._mass = mass

    def part_mass(self):
        return self._mass

    def part_inertia(self):
        return 0

    def part_lenght(self):
        return 0

    def CoM(self):
        return 0





