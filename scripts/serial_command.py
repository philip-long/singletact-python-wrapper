TIMEOUT=100

def GenerateWriteCommand(i2cAddress, ID, writeLocation, data):
    i = 0
    TIMEOUT = 100
    command = bytearray(len(data)+15)

    while (i < 4):
        command[i] = 255
        i += 1
    command[4] = i2cAddress
    command[5] = TIMEOUT
    command[6] = ID
    command[7] = 2
    command[8] = writeLocation
    command[9] = len(data)
    command[(10 + len(data))] = 255
    i = 0
    while (i < len(data)):
        command[(10 + i)] = data[i]
        i += 1
    i = 0
    while (i < 4):
        command[(11 + i) + len(data)] = 254
        i += 1
    return command

def GenerateReadCommand(i2cAddress, ID, readLocation, numToRead):
    command = bytearray(16)
    i = 0
    TIMEOUT = 100
    while (i < 4):
        command[i] = 0xFF
        i += 1
    command[4] = i2cAddress
    command[5] = TIMEOUT
    command[6] = ID
    command[7] = 0x01
    command[8] = readLocation
    command[9] = numToRead
    command[10] = 0xFF
    i = 0
    while (i < 4):
        command[(11 + i)] = 0xFE
        i += 1
    return command

def GenerateToggleCommand(i2cAddress, ID, writeLocation, data):
    i = 0
    command = bytearray(16 + 15)

    while (i < 4):
        command[i] = 255
        i += 1
    command[4] = i2cAddress
    command[5] = TIMEOUT
    command[6] = ID
    command[7] = 3
    command[8] = data
    command[9] = 16
    command[(10 + 16)] = 255
    i = 0
    while (i < 16):
        command[(10 + i)] = 7
        i += 1
    i = 0
    while (i < 4):
        command[((11 + i) + 16)] = 254
        i += 1
    return command