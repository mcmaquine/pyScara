from ctypes import sizeof
from MR_JE_C import *
from pyModbusTCP.client import ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.compat import iteritems



def main():
    c = ModbusClient(host='10.8.0.201', port=502, unit_id=255)
    c.open()

    result = c.read_holding_registers(index['MR_POSITION_ACTUAL_VALUE'], 2)
    print('Return size '+str(len(result)))
    print('Value read'+str(result))
    decoder = BinaryPayloadDecoder.fromRegisters(result, byteorder=Endian.Big)

    #print(decoder.decode_8bit_int())

    uint8_list = []
    for data in range(len(result)):
        print( data )
        decoder.skip_bytes(1)
        uint8_list.append( decoder.decode_8bit_int())

    print("-" * 60)
    print("Decoded Data")
    print("-" * 60)

    print( uint8_list )
    
    #client = ModbusTcpClient('10.8.0.201')
    #client.connect()

    #J1 = MR_JE_C( cli = client )

    #print( J1.set_mode('MR_POINT_TABLE_MODE'))
    #print( J1.get_mode())
    #print( 'Status word '+hex(J1.get_status_word()))

    #client.close()
    c.close()


if __name__ == '__main__':
    main()