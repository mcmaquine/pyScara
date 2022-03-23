def print_bits( data, total_bits = 16 ):
    if data != None:
        binary = bin(data)[2:0].zfill(16)
        print(binary)
    else:
        print('No data')