from MR_JE_C import *
from pymodbus.client.sync import ModbusTcpClient
import utils

def main():
    #client = ModbusTcpClient('10.8.0.201')
    #client.connect()

    #J1 = MR_JE_C( cli = client )

    #utils.print_bits( J1.get_status_word() )
    #result = client.read_holding_registers( index['MR_STATUS_WORD'], 1, unit=255)
    #print( hex( result.registers[0] ))
    #client.close()

    print( utils.set_bit( 0, bits['BIT_0'], bits['BIT_1'] ))

if __name__ == '__main__':
    main()