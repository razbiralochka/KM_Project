import numpy as np
from parts import *

class Stage:
    def __init__(self):
        self._stage_parts = list()
        self._lenght_of_parts = [0]
        self._stage_inertia = 0
    def add_part_to_stage(self, part):
        self._stage_parts.append(part)
        self._lenght_of_parts.append(part.part_lenght())


    def stage_mass(self):
        res = sum([part.part_mass() for part in self._stage_parts])
        return res
    def stage_lenght(self):
        res = sum([part.part_lenght() for part in self._stage_parts])
        return res
    def stage_inertia(self):
        for i, part in enumerate(self._stage_parts):
            self._stage_inertia+=part.part_inertia()+part.part_mass()*(self._lenght_of_parts[i]+part.CoM())**2
        return self._stage_inertia





upper_stage = Stage()

upper_stage.add_part_to_stage(Fairing(lenght=2.9,
                                      dia=1.7,
                                      mass=200))
upper_stage.add_part_to_stage(Payload(mass = 1000))
upper_stage.add_part_to_stage(Adapter(upper_dia = 1.7,
                                      lower_dia = 1.7,
                                      lenght = 0.6,
                                      mass = 200))
upper_stage.add_part_to_stage(Fuel_tank(dia=1.7,
                                        lenght=1.48,
                                        constr_mass=105.031,
                                        liquid_type="LOX"))
upper_stage.add_part_to_stage(Fuel_tank(dia=1.7,
                                        lenght=1.1,
                                        constr_mass=367.607,
                                        liquid_type="LNG"))
upper_stage.add_part_to_stage(Engine(imp=3700,
                                     cunsuption=15.9))
upper_stage.add_part_to_stage(Adapter(upper_dia=1.7,
                                      lower_dia=1.7,
                                      lenght=2.1,
                                      mass=200))

lower_stage = Stage()
lower_stage.add_part_to_stage(Fuel_tank(dia=1.7,
                                        lenght=8.39,
                                        constr_mass=435.02,
                                        liquid_type="LOX"))
lower_stage.add_part_to_stage(Fuel_tank(dia=1.7,
                                        lenght=6.21,
                                        constr_mass=1522.58,
                                        liquid_type="LNG"))
lower_stage.add_part_to_stage(Engine(imp=3300,
                                     cunsuption=120))
lower_stage.add_part_to_stage(Adapter(upper_dia=1.7,
                                      lower_dia=1.7,
                                      lenght=2.8,
                                      mass=300))


print("Масса 2-го РБ ", upper_stage.stage_mass())
print("Масса 1-го РБ ", lower_stage.stage_mass())
print("Длина ракеты: ",upper_stage.stage_lenght()+lower_stage.stage_lenght())

print("Инерция 2-го РБ: ",upper_stage.stage_inertia())
print("Инерция 1-го РБ: ",lower_stage.stage_inertia())