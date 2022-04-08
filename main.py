from ctypes import sizeof
from MR_JE_C import *
from pyModbusTCP.client import ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.compat import iteritems



        
client = ModbusTcpClient('10.8.0.201')
client2= ModbusTcpClient('10.8.2.202')

client.connect()
client2.connect()

J1 = MR_JE_C( cli = client )

print( 'status word '+ str( hex(J1.get_status_word())))
print( 'old control word '+ str(hex( J1.get_control_word()) ))
print('mode '+ str(J1.get_mode()))

J1.reset()

# J1.home()

client.close()
client2.close()