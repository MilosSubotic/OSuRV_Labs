#!/usr/bin/env python

from ULN2003 import *

motor3_pins = [11, 12, 13, 15]
motor2_pins = [16, 18, 22, 24]
motor1_pins = [19, 21, 23, 26]

motors = [ULN2003(motor1_pins), ULN2003(motor2_pins), ULN2003(motor3_pins)]

for i in range(0, 3):
    print(f"Moving motor {i} forwards")
    motors[i].step(n=1000)
    time.sleep(2)
    print(f"Moving motor {i} backwards")
    motors[i].step(n=-1000)
    time.sleep(2)
print("Done")
