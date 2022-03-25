from pymodbus.client.sync import ModbusTcpClient
import utils

index = {
    'MR_POINT_TABLE_OFFSET':    int(0x2801),
    'MR_TARGET_POINT_TABLE':    int(0x2D60),
    'MR_ERROR_CODE':		    int(0x603F),
    'MR_CONTROL_WORD':			int(0x6040),
    'MR_STATUS_WORD':           int(0x6041),
    'MR_MODE_OF_OPERATION':		int(0x6060),
    'MR_MODES_OPERATION_DISPLAY':int(0x6061),
    'MR_POSITION_ACTUAL_VALUE':	int(0x6064),
    'MR_HOMING_METHOD':          int(0x6098)
}

bits = {
    'BIT_0'     :   0x1,
    'BIT_1'     :   0x2,
    'BIT_2'     :   0x4,
    'BIT_3'     :   0x8,
    'BIT_4'     :   0x10,
    'BIT_5'     :   0x20,
    'BIT_6'     :   0x40,
    'BIT_7'     :   0x80,
    'BIT_8'     :   0x100,
    'BIT_9'     :   0x200,
    'BIT_10'    :   0x400,
    'BIT_11'    :   0x800,
    'BIT_12'    :   0x1000,
    'BIT_13'    :   0x2000,
    'BIT_14'    :   0x4000,
    'BIT_15'    :   0x8000,
    'NYBLE_0'   :   0xF,
    'NYBLE_1'   :   0xF0,
    'NYBLE_2'   :   0xF00,
    'NYBLE_3'   :   0xF000
}

class MR_JE_C:
    def __init__( self, **kwargs) -> None:
        self.cli = kwargs.get('cli', None)

    def get_status_word(self):
        result = utils.read( self.cli, index['MR_STATUS_WORD'] )
        if result != None:
            return result.registers[0]
        else:
            return None
    
    def is_servo_on( self ):
        result = self.get_status_word()
        pass