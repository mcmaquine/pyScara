from ctypes import sizeof
from MR_JE_C import *
from pyModbusTCP.client import ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.compat import iteritems



def main():
        
    client = ModbusTcpClient('10.8.0.201')
    client.connect()

    J1 = MR_JE_C( cli = client )

    print( 'status word '+ str( hex(J1.get_control_word())))
    print( 'old control word '+ str(hex( J1.get_status_word()) ))
    print('mode '+ str(J1.get_mode()))

    J1.servo_on()

    J1.reset()
    J1.reset_bits(bits['BIT_4'])

    print( 'old control word '+ str(hex( J1.get_status_word()) ))

    J1.home()

    client.close()


if __name__ == '__main__':
    main()