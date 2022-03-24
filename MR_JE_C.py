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

class MR_JE_C:
    def __init__( self, **kwargs) -> None:
        self.cli = kwargs.get('cli', None)

    def get_status_word(self):
        result = utils.read( self.cli, index['MR_STATUS_WORD'] )
        if result != None:
            return result.registers[0]
        else:
            return None