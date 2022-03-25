from MR_JE_C import *
from pymodbus.client.sync import *
import utils

def main():
    client = ModbusTcpClient('10.8.0.201')
    client.connect()

    J1 = MR_JE_C( cli = client )

    point = 2

    print( 'Angle: '+str(J1.get_pt_data(point).point_data) )
    print( 'Speed: '+str(J1.get_pt_data(point).speed) )
    print( 'Acc:   '+str(J1.get_pt_data(point).acceleration) )
    print( 'Decel: '+str(J1.get_pt_data(point).deceleration) )

    client.close()

if __name__ == '__main__':
    main()