import time
from MR_JE_C import *

J1 = MR_JE_C(ip='10.8.0.201')
time.sleep(0.5)
value = J1.get_status_word()
print('STATUS_WORD: {:02b}'.format(value))
value = J1.get_control_word()
print('CONTROL_WORD: {:02b}'.format(value))
J1.get_info()