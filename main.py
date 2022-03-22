from MR_JE_C import *
from pymodbus.client.sync import ModbusTcpClient

def main():
    cli = ModbusTcpClient('10.8.0.201')
    result = cli.read_holding_registers( index['MR_STATUS_WORD'], 1, unit=255 )
    cli.close()

    for data in result.registers:
        print( hex(data ))

    pass

if __name__ == '__main__':
    main()