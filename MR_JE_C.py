from pymodbus.client.sync import *
from pymodbus.payload import BinaryPayloadDecoder, BinaryPayloadBuilder
from pymodbus.constants import Endian
import utils

homing = {
    'MR_HOME_INTERRUPTED': 0x400,
    'MR_ILA': 0x800,
    'MR_HOME_COMPLETED': 0x1400,  # bit 10 e 12
    'MR_HOME_ERROR_SPEED': 0x2000,  # bit 13
    'MR_HOME_ERROR_SPEED_0': 0x2400  # bit 10 e 13
}

class MASK:
    BIT_0 = 0x0001
    BIT_1 = 0x0002
    BIT_2 = 0x0004
    BIT_3 = 0x0008
    BIT_4 = 0x0010
    BIT_5 = 0x0020
    BIT_6 = 0x0040
    BIT_7 = 0x0080
    BIT_8 = 0x0100
    BIT_9 = 0x0200
    BIT_10 = 0x0400
    BIT_11 = 0x0800
    BIT_12 = 0x1000
    BIT_13 = 0x2000
    BIT_14 = 0x4000
    BIT_15 = 0x8000
    NYBLE_0 = 0x0000F
    NYBLE_1 = 0x000F0
    NYBLE_2 = 0x00F00
    NYBLE_3 = 0x0F000
    ALL = 0xFFFF


class OP_MODES:
    NO_MODE = 0
    PROFILE_POSITION_MODE = 1
    PROFILE_VELOCITY_MODE = 2
    PROFILE_TORQUE_MODE = 3
    HOME_MODE = 6
    POSITION_CONTROL_MODE = -20
    SPEED_CONTROL_MODE = -21
    TORQUE_CONTROL_MODE = -22
    JOG_MODE = -100
    POINT_TABLE_MODE = -101
    INDEXER_MODE = -103


class MB_REG:
    # Modbus registers address, page 7- 12 (POSITIONING MODE)
    CONTROL_WORD = 0x6040
    STATUS_WORD = 0x6041
    ERROR_CODE = 0x603F
    MODE_OF_OPERATION = 0x6060
    MODE_OPERATION_DISPLAY = 0x6061
    POSITION_ACTUAL_VALUE = 0x6064
    HOMING_METHOD = 0x6098
    DEVICE_INFO = 0x1018
    POINT_TABLE_OFFSET = 0x2801
    TARGET_POINT_TABLE = 0x2d60
    PROF_ACELERATION = 0x6083
    PROF_DECELERATION = 0x6084
    QS_DECELERATION = 0x6085
    QS_OPTION_CODE = 0x605a
    PROF_VELOCITY = 0x6081
    MAX_PROF_VELOCITY = 0x607f
    MAX_MOTOR_SPEED = 0x6080
    TARGET_POSITION = 0x607a
    SOFTWARE_POSITION_LIMIT = 0x607d
    POINT_DEMANDED_VALUE = 0x2d68
    POINT_ACTUAL_VALUE = 0x2d69
    POINT_TABLE_ERROR = 0x2a43
    M_CODE_ACTUAL_VALUE = 0x2d6A
    GEAR_RATIO = 0x6091
    POLARITY = 0x607e
    FOLLOW_ERROR = 0x60f4
    ACTUAL_POSITION = 0x6064
    ACTUAL_VELOCITY = 0x606C
    ACTUAL_TORQUE = 0x6077


class CW_COMMANDS:
    # Control Word Commands, manual page 5-4
    SHUTDOWN = 0x0006
    SWITCH_ON = 0X0007
    DISABLE_VOLTAGE = 0x0000
    QUICK_STOP = 0x0002
    DISABLE_OPERATION = 0x0007
    ENABLE_OPERATION = 0x000F
    FAULT_RESET = 0x0010


class CW_BITS:
    # Control Word (6040) bit mapping.
    # Bits 4 to 6 are mode specific bits, in this case PP mode.
    SO = 0  # Switch On
    EV = 1  # Enable Voltage
    QS = 2  # Quick Stop
    EO = 3  # Enable Operation
    NEW_SET_POIT = 4  # New positioning parameters are obtained when this bit turns on
    CHANGE_SET = 5  # 0: Set of set-points;  1: Single set-point
    ABS_REL = 6  # 0: Absolut position command; 1: Relative position command
    HALT = 8  # 0: Positioning is executed 1: The servo motor stops according to Halt option code (605Dh)
    CHANGE_SET_POINT = 9  # 0: The next position start after the current position is completed. 1: Don´t stop


class SW_STATES:
    # Status Word States, manual page 5 - 5
    # The amplifier has a internal "status state machine" where all status can bre read with the
    # Statusword (6041h)
    #   (A) NOT_READY_TO_SWITCH_ON - Servo initialization in progress;
    #   (B) SWITCH_ON_DISABLED - In wait for forced stop reset;
    #   (C) READY_TO_SWITCH_ON - In Wait for main circuit charging (read-off)
    #   (D) SWITCHED_ON - Drive standby (servo-off)
    #   (E) OPERATION_ENABLE - In normal drive (servo-on)
    NOT_READY_TO_SWITCH_ON = 0x0000
    SWITCH_ON_DISABLED = 0x0020
    READY_TO_SWITCH_ON = 0x0031
    SWITCHED_ON = 0x0033
    OPERATION_ENABLE = 0x0037
    QUICK_STOP_ACTIVE = 0x0017
    FAULT_REACTION_ACTIVE = 0x000f
    FAULT = 0x0008
    MAIN_PWR_ON = 0x0010
    WARNING = 0x0080


class SW_BITS:
    # Status Word bit mapping
    RTSO = 0  # Ready to Switch On
    SO = 1  # Switched On
    OE = 2  # Operation Enabled
    FAULT = 3  # Fault
    VE = 4  # Voltage Enable
    QS = 5  # Quick stop
    SOD = 6  # Switch On Disabled
    W = 7  # Warning
    TR = 10  # Target reached
    ILA = 11  # Internal limit active
    SET_POINT_ACK = 12  # 0: Positioning completed; 1: Positioning being executed
    FOLLOW_ERROR = 13  # 0: No following error; 1: Following error


class PointData:
    n_entries = None
    point_data = None     # Coordinate
    speed = None
    acceleration = None
    deceleration = None
    dwell = None
    aux = None
    mcode = None


class MR_JE_C:
    def __init__(self, ip, port=502):
        self.client = ModbusTcpClient(ip, port)
        self.gear_ratio = 0
        self.gear_numerator = 0
        self.gear_denominator = 0
        self.mode = None
        self._open_com()

    def _open_com(self):
        # Open modbus  Modbus TCP communication
        try:
            result = self.client.connect()
            if result:
                print('Connection Opened: {}'.format(result))
            else:
                print('Connection failed')
        except Exception as e:
            print('open_com exception: %s' % e)

    def _read_float_register(self, address):
        result = self.client.read_holding_registers(address, 2, unit=255)
        value = BinaryPayloadDecoder.fromRegisters(result.registers,
                                                   byteorder=Endian.Big,
                                                   wordorder=Endian.Little).decode_32bit_float()
        return ('%.3f' % value).rstrip('.')

    def _read_register(self, address):
        result = self.client.read_holding_registers(address, 1, unit=255)
        value = BinaryPayloadDecoder.fromRegisters(result.registers,
                                                   byteorder=Endian.Big,
                                                   wordorder=Endian.Little).decode_16bit_uint()
        return value

    def _read_int8_register(self, address):
        result = self.client.read_holding_registers(address, 1, unit=255)
        value = BinaryPayloadDecoder.fromRegisters(result.registers,
                                                   byteorder=Endian.Big,
                                                   wordorder=Endian.Little)
        value.skip_bytes(1)
        return value.decode_8bit_int()

    def _write_register(self, address, value):
        result = self.client.write_registers(address, value, unit=255)
        return result

    def _decode_register_to_point(self, data):
        _decoder = BinaryPayloadDecoder.fromRegisters(data.registers,
                                                      byteorder=Endian.Big,
                                                      wordorder=Endian.Little)
        point = PointData()
        point.n_entries = _decoder.decode_16bit_int()
        point.point_data = _decoder.decode_32bit_int()
        point.speed = _decoder.decode_32bit_int()
        point.acceleration = _decoder.decode_32bit_int()
        point.deceleration = _decoder.decode_32bit_int()
        point.dwell = _decoder.decode_32bit_int()
        point.aux = _decoder.decode_32bit_int()
        point.mcode = _decoder.decode_32bit_int()
        return point

    def _encode_point_to_register(self, point):
        _payload = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Little)
        _payload.add_16bit_uint(point.n_entries)
        _payload.add_32bit_int(point.point_data)
        _payload.add_32bit_int(point.speed)
        _payload.add_32bit_int(point.acceleration)
        _payload.add_32bit_int(point.deceleration)
        _payload.add_32bit_int(point.dwell)
        _payload.add_32bit_int(point.aux)
        _payload.add_32bit_int(point.mcode)
        return _payload.to_registers()

    def get_info(self):
        data = self.client.read_holding_registers(MB_REG.DEVICE_INFO, 9, unit=255)
        print(data)
        if data is not None:
            print('Vendor ID        :' + str(hex(data.registers[1] | data.registers[2] << 16)))
            print('Product Code     :' + str(hex(data.registers[3] | data.registers[4] << 16)))
            print('Revision Number  :' + str(hex(data.registers[5] | data.registers[6] << 16)))
            print('Serial number    :' + str(hex(data.registers[7] | data.registers[8] << 16)))
        else:
            print('No data - must check connection')

    def get_status_word(self):
        result = self._read_register(MB_REG.STATUS_WORD)
        return result

    def get_control_word(self):
        result = self._read_register(MB_REG.CONTROL_WORD)
        return result

    def write_control_word(self, command):
        return self._write_register(MB_REG.CONTROL_WORD, command)

    def decode_status(self):
        status_word = self.get_status_word()
        print('STATUS_WORD: {:02b}'.format(status_word))
        status_word = status_word & 0x0F
        print('STATUS_WORD: {:02b}'.format(status_word))

        if status_word == SW_STATES.NOT_READY_TO_SWITCH_ON:
            print("Not ready to switch on")
        if status_word == SW_STATES.SWITCH_ON_DISABLED:
            print("Switch on disabled")
        if status_word == SW_STATES.READY_TO_SWITCH_ON:
            print("Ready to switch on")
        if status_word == SW_STATES.SWITCHED_ON:
            print("Switch on")
        if status_word == SW_STATES.OPERATION_ENABLE:
            print("Operation enable")
        if status_word == SW_STATES.QUICK_STOP_ACTIVE:
            print("Quick stop active")
        if status_word == SW_STATES.FAULT_REACTION_ACTIVE:
            print("Fault reaction active")
        if utils.check_bit_set(status_word, SW_BITS.FAULT):
            print("Fault ")

    def servo_on(self):
        self.write_control_word(CW_COMMANDS.ENABLE_OPERATION)

    def servo_off(self):
        self.write_control_word(CW_COMMANDS.DISABLE_OPERATION)

    def reset_bits(self, *bits):
        mask = 0
        result = self.get_control_word()
        print('Before reset: ' + str(hex(result)))

        for bit in bits:
            mask = mask | bit

        result = result & ~mask
        print('After resets ' + str(hex(result)))
        utils.write(self.cli, index['MR_CONTROL_WORD'], result)

    def set_bits(self, *bits):
        word = self.get_control_word()

        if word is not None:
            for bit in bits:
                word = word | bit
            self.cli.write(MR_CONTROL_WORD, word)
            return True
        else:
            return False

    def get_actual_position(self):
        words = utils.read(self.cli, index['MR_POSITION_ACTUAL_VALUE'], 2)
        if words is not None:
            decode = BinaryPayloadDecoder.fromRegisters(words.registers, byteorder=Endian.Big, wordorder=Endian.Little)
            position = decode.decode_32bit_int()
            return position
        else:
            return None

    def get_point_data(self, point_numb):
        # get point table data
        point = PointData()
        if point_numb < 0 or point_numb > 255:
            return None
        try:
            data = self.client.read_holding_registers(MB_REG.POINT_TABLE_OFFSET + point_numb, 15, unit=255)
            print("get_poit_data {}: {}".format(point_numb, data.registers))
        except Exception as e:
            print('get_point_data exception: %s' % e)

        if data is not None:
            point = self._decode_register_to_point(data)
            return point
        else:
            return None

    def set_point_data(self, point_numb, cord, speed, accel, decel):
        # Set point table data
        point = PointData()
        if point_numb < 0 or point_numb > 255:
            return None
        point.n_entries = 7
        point.point_data = cord
        point.speed = speed
        point.acceleration = accel
        point.deceleration = decel
        point.dwell = 0
        point.aux = 0
        point.mcode = 0
        payload = self._encode_point_to_register(point)
        print("set_point_data {}: {}".format(point_numb, payload))
        try:
            self.client.write_registers(MB_REG.POINT_TABLE_OFFSET + point_numb, payload, unit=255)
        except Exception as e:
            print('set_point_data exception: %s' % e)

    def get_electronic_gear_ratio(self):
        data = utils.read(self.cli, index['MR_GEAR_RATIO'], 5)
        if data is not None:
            self.gear_ratio = data.registers[0]
            self.gear_numerator = data.registers[1] | data.registers[2] << 16
            self.gear_denominator = data.registers[3] | data.registers[4] << 16

            return [self.gear_ratio, self.gear_numerator, self.gear_denominator]
        else:
            return None

    def set_mode(self, mode):
        if mode in modes.keys():
            status = utils.write(self.cli, index['MR_MODE_OF_OPERATION'], modes[mode])
            if status is None:
                return False
            else:
                return True
        else:
            return False

    def get_mode(self):
        result = self._read_int8_register(MB_REG.MODE_OPERATION_DISPLAY)
        return result

    def home(self):
        mode = self.get_mode()
        if mode is None:
            return False
        elif mode is not modes['MR_HOME_MODE']:
            self.set_mode('MR_HOME_MODE')

        self.set_bits(bits['BIT_4'])

        # send bit 4 set to start home

        status_word = self.get_status_word()

        while True:
            if (status_word & homing['MR_HOME_COMPLETED']) is homing['MR_HOME_COMPLETED']:
                retorno = True
                break

            elif (status_word & homing['MR_HOME_ERROR_SPEED_0']) is homing['MR_HOME_ERROR_SPEED_0'] or \
                    (status_word & homing['MR_HOME_ERROR_SPEED']) is homing['MR_HOME_ERROR_SPEED'] or \
                    (status_word & homing['MR_HOME_INTERRUPTED']) is homing['MR_HOME_INTERRUPTED']:
                retorno = False
                break

            status_word = self.get_status_word()
            print('status word' + str(hex(status_word)))

        self.reset_bits(bits['BIT_4'])

        return retorno

    def reset(self):
        status = self.get_control_word()
        if status is not None:
            self.set_bits(bits['BIT_7'])
            ret = self.get_control_word()
            if (ret & bits['BIT_7']) is bits['BIT_7']:
                self.reset_bits(bits['BIT_7'])

