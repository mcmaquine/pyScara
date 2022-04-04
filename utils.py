from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder

# print in bit frmat (NOT FINISHED)
def print_bits( data, total_bits = 16 ):
    if data is not None:
        binary = str(bin(data)).replace('0b','')
        binary.zfill( total_bits )
    else:
        print('No data')

#modbusTcp read a holding register
def read ( ModbusTCPClient, index, count) -> int:
    if ModbusTCPClient != None:
        if ModbusTCPClient.is_socket_open():
            return ModbusTCPClient.read_holding_registers( index, count, unit=255 )
        else: #try to connect one time
            ModbusTCPClient.connect()
            if ModbusTCPClient.is_socket_open():
                return ModbusTCPClient.read_holding_registers( index, count, unit=255 )
            else:
                return None
    else:
        return None
    
def write ( ModbusTCPClient, index, word):
    if ModbusTCPClient != None:
        if ModbusTCPClient.is_socket_open():
            
            print(word)
            builder = BinaryPayloadBuilder(wordorder=Endian.Big, byteorder=Endian.Big)
            builder.add_16bit_int(word)
            payload = builder.build()

            print( payload )

            return ModbusTCPClient.write_registers( index, payload, unit=255 )
        else:
            ModbusTCPClient.connect()
            if ModbusTCPClient.is_socket_open():
                return ModbusTCPClient.write_registers( index, word, unit=255 )
            else:
                return None
    else:
        return None

# set a bit (or bits) in a word
def set_bit( word, *bits) -> int:
    result = word
    
    for bit in bits:
        result = result | bit
    return result

# reset a bit (or bits) in a word
def reset_bit( word, *bits) -> int:
    mask = 0
    result = word
    
    for bit in bits:
        mask = mask | bit
    
    result = result & ~mask

    return result
