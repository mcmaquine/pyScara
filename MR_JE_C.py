import time

from pymodbus.client.sync import *
from pymodbus.payload import BinaryPayloadDecoder, BinaryPayloadBuilder
from pymodbus.constants import Endian
import utils

class HOMING:
    HOME_INTERRUPTED = 0x400
    ILA = 0x800
    HOME_COMPLETED = 0x1400     # bit 10 e 12
    HOME_ERROR_SPEED = 0x2000   # bit 13
    HOME_ERROR_SPEED_0 = 0x2400 # bit 10 e 13

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
    POINT_TABLE_MODE = 155 #(=-101)
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
    PROF_ACCELERATION = 0x6083
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
    # Bits 4 to 6 are mode specific bits, in this case PT (Point Table) mode.
    SO = 0  # Switch On
    EV = 1  # Enable Voltage
    QS = 2  # Quick Stop
    EO = 3  # Enable Operation
    NEW_SET_POIT = 4        # New positioning parameters are obtained when this bit turns on
    DIRECTION = 5           # 0: Set of set-points;  1: Single set-point
    HALT = 8                # 0: Positioning is executed 1: The servo motor stops according to Halt option code (605Dh)


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
    RTSO = 0    # Ready to Switch On
    SO = 1      # Switched On
    OE = 2      # Operation Enabled
    FAULT = 3   # Fault
    VE = 4      # Voltage Enable
    QS = 5      # Quick stop
    SOD = 6     # Switch On Disabled
    W = 7       # Warning
    TR = 10     # Target reached
    ILA = 11    # Internal limit active
    SET_POINT_ACK = 12 # 0: Positioning completed; 1: Positioning being executed
    FOLLOW_ERROR = 13  # 0: No following error; 1: Following error


class PointData:
    n_entries = None
    point_data = None     # Coordinate
    speed = None
    acceleration = None
    deceleration = None
    dwell = None
    aux = None
    mCode = None


class MR_JE_C:
    def __init__(self, ip, port=502):
        self.client = ModbusTcpClient(ip, port)
        self._open_com()

    def _open_com(self):
        # Open Modbus TCP communication
        try:
            result = self.client.connect()
            if result:
                print('Connection Opened: {}'.format(result))
            else:
                print('Connection failed')
        except Exception as e:
            print('open_com exception: %s' % e)

    def _read_float_register(self, address):
        try:
            result = self.client.read_holding_registers(address, 2, unit=255)
            value = BinaryPayloadDecoder.fromRegisters(result.registers,
                                                       byteorder=Endian.Big,
                                                       wordorder=Endian.Little).decode_32bit_float()
            return ('%.3f' % value).rstrip('.')
        except Exception as e:
            print('_read_float_register exception: %s' % e)
        return None

    def _read_int32_register(self, address):
        try:
            result = self.client.read_holding_registers(address, 2, unit=255)
            value = BinaryPayloadDecoder.fromRegisters(result.registers,
                                                       byteorder=Endian.Big,
                                                       wordorder=Endian.Little).decode_32bit_int()
            return value
        except Exception as e:
            print('_read_int32_register exception: %s' % e)
        return None

    def _read_register(self, address):
        try:
            result = self.client.read_holding_registers(address, 1, unit=255)
            value = BinaryPayloadDecoder.fromRegisters(result.registers,
                                                       byteorder=Endian.Big,
                                                       wordorder=Endian.Little).decode_16bit_uint()
            return value
        except Exception as e:
            print('_read_register exception: %s' % e)
        return None

    def _read_int8_register(self, address):
        result = self.client.read_holding_registers(address, 1, unit=255)
        value = BinaryPayloadDecoder.fromRegisters(result.registers,
                                                   byteorder=Endian.Big,
                                                   wordorder=Endian.Little)
        value.skip_bytes(1)
        return value.decode_8bit_int()

    def _write_registers(self, address, value):
        try:
            self.client.write_registers(address, value, unit=255)
            return False
        except Exception as e:
            print('_write_register exception: %s' % e)
        return True

    @staticmethod
    def _decode_register_to_point(data):
        # Decodes the information from Modbus registers to a PointDada class
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
        point.mCode = _decoder.decode_32bit_int()
        return point

    @staticmethod
    def _encode_point_to_register(point):
        _payload = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Little)
        _payload.add_16bit_uint(point.n_entries)
        _payload.add_32bit_int(point.point_data)
        _payload.add_32bit_int(point.speed)
        _payload.add_32bit_int(point.acceleration)
        _payload.add_32bit_int(point.deceleration)
        _payload.add_32bit_int(point.dwell)
        _payload.add_32bit_int(point.aux)
        _payload.add_32bit_int(point.mCode)
        return _payload.to_registers()

    def get_info(self):
        data = self.client.read_holding_registers(MB_REG.DEVICE_INFO, 9, unit=255)
        _decoder = BinaryPayloadDecoder.fromRegisters(data.registers,
                                                      byteorder=Endian.Big,
                                                      wordorder=Endian.Little)
        if data is not None:
            print('ID object      :{}'.format(_decoder.decode_16bit_uint()))
            print('Vendor ID      :{}'.format(_decoder.decode_32bit_uint()))
            print('Product Code   :{}'.format(_decoder.decode_32bit_uint()))
            print('Revision Number:{}'.format(_decoder.decode_32bit_uint()))
            print('Serial number  :{}'.format(_decoder.decode_32bit_uint()))
        else:
            print('No data - must check connection')

    def get_status_word(self):
        result = self._read_register(MB_REG.STATUS_WORD)
        return result

    def get_control_word(self):
        result = self._read_register(MB_REG.CONTROL_WORD)
        return result

    def write_control_word(self, command):
        return self._write_registers(MB_REG.CONTROL_WORD, command)

    def decode_status(self):
        status_word = self.get_status_word()
        print('STATUS_WORD: {:02b}'.format(status_word))
        status_word = status_word & 0x0F
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
        print("Servo On")
        self.write_control_word(CW_COMMANDS.ENABLE_OPERATION)

    def servo_off(self):
        print("Servo Off")
        self.write_control_word(CW_COMMANDS.DISABLE_OPERATION)

    def get_point_data(self, point_numb):
        # get point table data
        point = PointData()
        if point_numb < 0 or point_numb > 255:
            return None
        try:
            data = self.client.read_holding_registers(MB_REG.POINT_TABLE_OFFSET + (point_numb - 1), 15, unit=255)
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
        if point_numb < 1 or point_numb > 255:
            return None
        point.n_entries = 7
        point.point_data = cord
        point.speed = speed
        point.acceleration = accel
        point.deceleration = decel
        point.dwell = 0
        point.aux = 0
        point.mCode = 0
        payload = self._encode_point_to_register(point)
        print("set_point_data {}: {}".format(point_numb, payload))
        self._write_registers(MB_REG.POINT_TABLE_OFFSET + (point_numb - 1), payload)

    def get_electronic_gear_ratio(self):
        # Get Gear Ratio oncfigured o MR Configurator
        # This data is important to know about servo parametrization
        # related to mechanics
        data = utils.read(self.cli, MB_REG.gear_ratio, 5)
        if data is not None:
            self.gear_ratio = data.registers[0]
            self.gear_numerator = data.registers[1] | data.registers[2] << 16
            self.gear_denominator = data.registers[3] | data.registers[4] << 16

            return [self.gear_ratio, self.gear_numerator, self.gear_denominator]
        else:
            return None

    def set_mode(self, mode):
        self._write_registers(MB_REG.MODE_OF_OPERATION, mode)

    def get_mode(self):
        result = self._read_int8_register(MB_REG.MODE_OPERATION_DISPLAY)
        return result

    def get_home_method(self):
        result = self._read_register(MB_REG.HOMING_METHOD)
        print(result)

    def home(self):
        mode = self.get_mode()
        if mode is not OP_MODES.HOME_MODE:
            self.set_mode(OP_MODES.HOME_MODE)

        control_word = self.get_control_word()
        control_word = utils.set_bit_reg(control_word, CW_BITS.NEW_SET_POIT)
        # send bit 4 set to start home
        self.write_control_word(control_word)
        result = False
        while True:
            status_word = self.get_status_word()
            if (status_word & HOMING.HOME_COMPLETED) == HOMING.HOME_COMPLETED:
                result = True
                break
        # send bit 4 clear to stop home
        control_word = utils.clear_bit_reg(control_word,CW_BITS.NEW_SET_POIT)
        self.write_control_word(control_word)
        return result

    def execute_point(self, point_numb):
        # Set point
        self._write_registers(MB_REG.TARGET_POINT_TABLE, point_numb)

        # Execute moviment
        control_word = self.get_control_word()
        control_word = utils.set_bit_reg(control_word, CW_BITS.NEW_SET_POIT)
        #print("control_word: {:016b}".format(control_word))
        self.write_control_word(control_word)

        #time.sleep(0.3)

        control_word = self.get_control_word()
        control_word = utils.clear_bit_reg(control_word, CW_BITS.NEW_SET_POIT)
        #print("control_word: {:016b}".format(control_word))
        self.write_control_word(control_word)

    def get_actual_position(self):
        return self._read_int32_register(MB_REG.ACTUAL_POSITION)

    def get_point_actual_value(self):
        return self._read_int32_register(MB_REG.POINT_ACTUAL_VALUE)
