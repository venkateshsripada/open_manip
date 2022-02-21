#!/usr/bin/env python

import numpy as np

modes = ["no GUI", "GUI"]
auto = ["autonomy", "teleop"]
mixed = ["GUI autonomy", "GUI teleop", "no GUI autonomy", "no GUI teleop"]
# object_order = ["cup","small cup","solid cup","cup","small cup","solid cup","cup","small cup","solid cup","cup"]
object_order = ["cup", "solid", "cup", "solid", "cup"]

np.random.shuffle(mixed)
np.random.shuffle(object_order)
print(mixed)
print(" ")
print(object_order)




