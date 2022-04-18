from ctypes import sizeof
from MR_JE_C import *
from pyModbusTCP.client import ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.compat import iteritems



        
client = ModbusTcpClient('10.8.0.201', port=502)
client2= ModbusTcpClient('10.8.2.202', port=502)

client.connect()
client2.connect()

J1 = MR_JE_C( cli = client )

print( 'Encoder: ', J1.get_actual_position() )
print( 'Mode of opeation: ', J1.get_mode())
J1.servo_off()

J1.reset_bits( bits['BIT_4'])
#J1.reset()
J1.set_mode( modes['MR_JOG_MODE'])

#J1.home()

client.close()
client2.close()