from MR_JE_C import *
from pymodbus.client.sync import *
from pyModbusTCP.client import ModbusClient


def main():
    c = ModbusClient(host='10.8.0.201', port=502, unit_id=255)
    c.open()

    result = c.read_holding_registers(index['MR_MODES_OPERATION_DISPLAY'], 1)
    print(result[0])
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