import utils

index = {
    'MR_DEVICE_INFO': 0x1018,
    'MR_POINT_TABLE_OFFSET': 0x2801,
    'MR_TARGET_POINT_TABLE': 0x2D60,
    'MR_ERROR_CODE': 0x603F,
    'MR_CONTROL_WORD': 0x6040,
    'MR_STATUS_WORD': 0x6041,
    'MR_MODE_OF_OPERATION': 0x6060,
    'MR_MODES_OPERATION_DISPLAY': 0x6061,
    'MR_POSITION_ACTUAL_VALUE': 0x6064,
    'MR_GEAR_RATIO': 0x6091,
    'MR_HOMING_METHOD': 0x6098
}

status = {
    'MR_SWITCHED_ON': 0x7,
    'MR_READY_TO_SWITCH_ON': 0x1,
    'MR_WARNING': 0x80
}

bits = {
    'BIT_0': 0x1,
    'BIT_1': 0x2,
    'BIT_2': 0x4,
    'BIT_3': 0x8,
    'BIT_4': 0x10,
    'BIT_5': 0x20,
    'BIT_6': 0x40,
    'BIT_7': 0x80,
    'BIT_8': 0x100,
    'BIT_9': 0x200,
    'BIT_10': 0x400,
    'BIT_11': 0x800,
    'BIT_12': 0x1000,
    'BIT_13': 0x2000,
    'BIT_14': 0x4000,
    'BIT_15': 0x8000,
    'NYBLE_0': 0xF,
    'NYBLE_1': 0xF0,
    'NYBLE_2': 0xF00,
    'NYBLE_3': 0xF000
}


class MR_JE_C:
    def __init__(self, **kwargs) -> None:
        self.cli = kwargs.get('cli', None)
        self.gear_ratio = 0
        self.gear_numerator = 0
        self.gear_denominator = 0
        self.mode = None

    def get_info(self):
        data = utils.read(self.cli, index['MR_DEVICE_INFO'], 9)
        if data is not None:
            print('Vendor ID        :' + str(hex(data.registers[1] | data.registers[2] << 16)))
            print('Product Code     :' + str(hex(data.registers[3] | data.registers[4] << 16)))
            print('Revision Number  :' + str(hex(data.registers[5] | data.registers[6] << 16)))
            print('Serial number    :' + str(hex(data.registers[7] | data.registers[8] << 16)))
        else:
            print('No data - must check connection')

    def get_status_word(self) -> int:
        result = utils.read(self.cli, index['MR_STATUS_WORD'], 1)
        if result is not None:
            return result.registers[0]
        else:
            return None

    def get_control_word(self, **kwargs) -> int:
        result = utils.read(self.cli, index['MR_CONTROL_WORD'], 1)
        if result is not None:
            return result.registers[0]
        else:
            return None

    def is_servo_on(self):
        word = self.get_status_word()
        if word is not None:
            return None
        else:
            if word & status['MR_SWITCHED_ON'] == status['MR_SWITCHED_ON']:
                return True
            else:
                return False

    def servo_on(self):
        word = self.get_control_word()
        word = utils.set_bit(word, bits['NYBLE_0'])
        return utils.write(self.cli, index['MR_CONTROL_WORD'], word)

    def servo_off(self):
        word = self.get_control_word()
        word = utils.reset_bit(word, bits['NYBLE_0'])
        return utils.write(self.cli, index['MR_CONTROL_WORD'], word)

    def get_actual_position(self):
        words = utils.read(self.cli, index['MR_POSITION_ACTUAL_VALUE'], 2)
        if words is not None:
            position = words.registers[1]
            position = position << 16
            position = position | words.registers[0]
            return position
        else:
            return None

    # get point table data
    def get_pt_data(self, point):
        if point < 1 or point > 255:
            return None

        data = utils.read(self.cli, index['MR_POINT_TABLE_OFFSET'] + point - 1, 15)

        if data is not None:
            point_table = PointTable()
            point_table.n_entries = data.registers[0]
            point_table.point_data = data.registers[1] | (data.registers[2] << 16)
            point_table.speed = data.registers[3] | (data.registers[4] << 16)
            point_table.acceleration = data.registers[5] | (data.registers[6] << 16)
            point_table.deceleration = data.registers[7] | (data.registers[8] << 16)
            point_table.dwell = data.registers[9] | (data.registers[10] << 16)
            point_table.aux = data.registers[11] | (data.registers[12] << 16)
            point_table.mcode = data.registers[13] | (data.registers[14] << 16)

            return point_table
        else:
            return None

    def get_electronic_gear_ratio(self):
        data = utils.read(self.cli, index['MR_GEAR_RATIO'], 5)
        if data is not None:
            self.gear_ratio = data.registers[0]
            self.gear_numerator = data.registers[1] | data.registers[2] << 16
            self.gear_denominator = data.registers[3] | data.registers[4] << 16

            return [self.gear_ratio, self.gear_numerator, self.gear_denominator]
        else:
            return None

    # precisa finalizar
    def set_pt_data(self, point):
        pass


class PointTable:
    def __init__(self, **kwargs):
        self.point = kwargs.get('point', 0)
        self.n_entries = kwargs.get('n_entries', 0)
        self.point_data = kwargs.get('point_data', 0)
        self.speed = kwargs.get('speed', 0)
        self.acceleration = kwargs.get('acceleration', 0)
        self.deceleration = kwargs.get('deceleration', 0)
        self.dwell = kwargs.get('dwell', 0)
        self.aux = kwargs.get('aux', 0)
        self.mcode = kwargs.get('mcode', 0)

    # return all field in a list ready to write
    def get_list(self):
        data = list(15)

        data[0] = self.point

        data[1] = self.point_data & 0xFFFF
        data[2] = self.point_data >> 16

        data[3] = self.speed & 0x0000FFFF
        data[4] = self.speed >> 16

        data[5] = self.acceleration & 0x0000FFFF
        data[6] = self.acceleration >> 16

        data[7] = self.deceleration & 0x0000FFFF
        data[8] = self.deceleration >> 16

        data[9] = self.dwell & 0x0000FFFF
        data[10] = self.dwell >> 16

        data[11] = self.aux & 0x0000FFFF
        data[12] = self.aux >> 16

        data[13] = self.mcode & 0x0000FFFF
        data[14] = self.mcode >> 16

        return data
