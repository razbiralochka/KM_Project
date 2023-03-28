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
        print(self._lenght_of_parts)

    def stage_mass(self):
        res = sum([part.part_mass() for part in self._stage_parts])
        return res
    def stage_lenght(self):
        res = sum([part.part_lenght() for part in self._stage_parts])
        return res
    def stage_inertia(self):
        for i, part in enumerate(self._stage_parts):
            self._stage_inertia+=part.part_inertia()+part.part_mass()*self._lenght_of_parts[i]**2
        return self._stage_inertia





upper_stage = Stage()

upper_stage.add_part_to_stage(Fairing(2.9, 1.7, 200))
upper_stage.add_part_to_stage(Payload(1000))
upper_stage.add_part_to_stage(Adapter(1.7, 1.7, 0.6, 200))
upper_stage.add_part_to_stage(Fuel_tank(1.7, 1.48, 111, liquid_type="LOX"))
upper_stage.add_part_to_stage(Fuel_tank(1.7, 1.1, 111,  liquid_type="LNG"))
upper_stage.add_part_to_stage(Engine(3700, 15.9))
upper_stage.add_part_to_stage(Adapter(1.7,1.7,2.1,500))

lower_stage = Stage()
lower_stage.add_part_to_stage(Fuel_tank(1.7,8.39,333,liquid_type="LOX"))
lower_stage.add_part_to_stage(Fuel_tank(1.7, 6.21, 222,  liquid_type="LNG"))
lower_stage.add_part_to_stage(Engine(3700,20))
lower_stage.add_part_to_stage(Adapter(1.7,1.7,2.8,500))



print(lower_stage.stage_mass())
print(lower_stage.stage_lenght())
print(lower_stage.stage_inertia())
