import numpy as np
from parts import *

class Stage:
    def __init__(self):
        self._stage_parts = list()

    def add_part_to_stage(self, part):
        self._stage_parts.append(part)


upper_stage = Stage()

upper_stage.add_part_to_stage(Fairing(2.9, 1.7, 200), 0)
upper_stage.add_part_to_stage(Payload(1000), 2.9)
upper_stage.add_part_to_stage(Adapter(1.7, 1.7, 0.6, 200))
upper_stage.add_part_to_stage(Fuel_tank(1.7, 1.48, 0.3, liquid_type="LOX"))
upper_stage.add_part_to_stage(Fuel_tank(1.7, 1.1, 0.2,  liquid_type="LNG"))
upper_stage.add_part_to_stage(Engine(3700, 15.9))



