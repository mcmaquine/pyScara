from MR_JE_C import *
from pymodbus.client.sync import *


def main():
    client = ModbusTcpClient('10.8.0.201')
    client.connect()

    J1 = MR_JE_C(cli=client)

    print(J1.get_actual_position())

    client.close()


if __name__ == '__main__':
    main()
