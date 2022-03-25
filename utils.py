def print_bits( data, total_bits = 16 ):
    if data != None:
        binary = str(bin(data)).replace('0b','')
        binary.zfill( total_bits )
    else:
        print('No data')

#modbusTcp read a holding register
def read ( ModbusTCPClient, index) -> int:
    if ModbusTCPClient != None:
        if ModbusTCPClient.is_socket_open():
            return ModbusTCPClient.read_holding_registers( index, 1, unit=255 )
        else: #try to connect one time
            ModbusTCPClient.connect()
            if ModbusTCPClient.is_socket_open():
                return ModbusTCPClient.read_holding_registers( index, 1, unit=255 )
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