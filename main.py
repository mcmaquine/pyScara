import time
import utils
from MR_JE_C import *

J1 = MR_JE_C(ip='10.8.0.201')
J2 = MR_JE_C(ip='10.8.0.202')

print("Servo 1, Base")
J1.get_info()
print("Servo 2, Braco")
J2.get_info()

J1.servo_on()
J2.servo_on()

#J1.servo_off()
#J2.servo_off()

time.sleep(0.5)

J1.set_point_data(1, 9000, 10000, 100, 100)
J1.set_point_data(2, 13500, 10000, 100, 100)
J1.set_point_data(3, 0, 10000, 100, 100)

J2.set_point_data(1, 0, 10000, 100, 100)
J2.set_point_data(2, 5000, 10000, 100, 100)
J2.set_point_data(3, -5000, 10000, 100, 100)

#J1.home()
#print("Act position J1: {}".formtat(J2.get_actual_position()))
#J2.home()
#print("Act position J2: {}".format(J2.get_actual_position()))

J1.set_mode(OP_MODES.POINT_TABLE_MODE)
print("Mode J1: {}".format(J1.get_mode()))
J2.set_mode(OP_MODES.POINT_TABLE_MODE)
print("Mode J2: {}".format(J2.get_mode()))


while True:
    J1.execute_point(1)
    J2.execute_point(2)
    time.sleep(2)
    J1.execute_point(2)
    J2.execute_point(1)
    time.sleep(2)
    #J1.execute_point(3)
    #time.sleep(2)

