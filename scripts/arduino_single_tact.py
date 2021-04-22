import serial
import time
import serial_command
import os
import rospy


class ArduinoSingleTactDriver:

    def __init__(self, port_name):
        rospy.loginfo("Arduino communication starting")
        self.cmdItr_ = 0
        self.TIMESTAMP_SIZE = 4
        self.I2C_ID_BYTE = 6
        self.I2C_TIMESTAMP = 7
        self.I2C_TOPC_NBYTES = 11
        self.I2C_START_OF_DATA = 12
        self.MINIMUM_FROMARDUINO_PACKET_LENGTH = 15
        self.TIMESTAMP_SIZE = 4
        self.serialPort_ = serial.Serial(port_name)
        self.serialPort_.baudrate = 115200
        if os.name =='nt': # If we're in windows we need to set the buffers
            self.serialPort_.set_buffer_size(48, 16)  # receiving and transmitting
        self.serialPort_.setDTR(True)
        time.sleep(1)
        self.serialPort_.setDTR(False)
        time.sleep(1)
        self.serialPort_.setRTS(True)

        self.incommingSerialBuffer_ = bytearray()
        self.isUSB = False

    def WriteToMainRegister(self, toSend, location, i2CAddress):
        if len(toSend) > 28:
            print("Trying to write a packet that is too large- Error")
            raise Exception('Error - Trying to write too much len(toSend) < 32 but is ', len(toSend))
            return False
        if (len(toSend) + location) > 128:
            raise Exception('Error - Trying to write in a read only len(toSend) + location  < 128'
                            ' but is', len(toSend) + location)

            return False

        if (self.cmdItr_ < 255):
            self.cmdItr_ = self.cmdItr_ + 1
        else:
            self.cmdItr_ = 0
        cmdToArduino = serial_command.GenerateWriteCommand(i2CAddress, self.cmdItr_, location, toSend)
        self.serialPort_.write(cmdToArduino)
        acknowledged = False
        attempts = 20
        time.sleep(0.005)

        while (acknowledged == False) and (attempts > 0):
            cmdFromArduino = self.ProcessSerialBuffer()
            if cmdFromArduino != None:
                if cmdToArduino[self.I2C_ID_BYTE] == cmdFromArduino[self.I2C_ID_BYTE]:
                    acknowledged = True
                    return True
                else:
                    print("Comms Error - Failed Acknoledge Write Command", "Communications Error")
                    acknowledged = True
                    return False
            else:
                time.sleep(0.005)
                attempts -= 1
        return False

    def ReadFromMainRegister(self, location, nBytes, i2CAddress):
        if nBytes > 32:
            print("Error - Trying to read too much")
            raise Exception('Error - Trying to read too much nBytes should be >32 but is ',nBytes)
            return None
        if (location + nBytes) > 191:
            raise Exception('Error - Trying to read off the end of the main register location + nBytes should be < '
                            '191 but is',location + nBytes)
            return None
        if(self.cmdItr_<255):
            self.cmdItr_ = self.cmdItr_ + 1
        else:
            self.cmdItr_=0
        cmdToArduino = serial_command.GenerateReadCommand(i2CAddress, self.cmdItr_, location, nBytes)


        try:
            self.serialPort_.write(cmdToArduino)
        except Exception:
            print("Write failed")
            return None

        acknowledged = False
        attempts = 50
        time.sleep(0.05)

        while (acknowledged == False) and (attempts > 0):

            cmdFromArduino = self.ProcessSerialBuffer()  # Philip
            if cmdFromArduino is not None:
                if cmdToArduino[self.I2C_ID_BYTE] == cmdFromArduino[self.I2C_ID_BYTE]:
                    acknowledged = True
                    toReturn = bytearray(nBytes + self.TIMESTAMP_SIZE)

                    # I'm not sure about this
                    # Array.Copy(cmdFromArduino, self.I2C_TIMESTAMP, toReturn, 0, self.TIMESTAMP_SIZE)
                    toReturn[0:self.TIMESTAMP_SIZE] \
                        = cmdFromArduino[self.I2C_TIMESTAMP:(self.I2C_TIMESTAMP + self.TIMESTAMP_SIZE)]
                    # Array.Copy(cmdFromArduino, self.I2C_START_OF_DATA, toReturn, self.TIMESTAMP_SIZE, nBytes)
                    toReturn[self.TIMESTAMP_SIZE:(self.TIMESTAMP_SIZE + nBytes)] \
                        = cmdFromArduino[self.I2C_START_OF_DATA:(self.I2C_START_OF_DATA + nBytes)]
                    return toReturn
                else:
                    print("Comms error - failed ack")
                    acknowledged = True
                    return None
            else:
                time.sleep(0.005)
                attempts -= 1
        return None

    def ProcessSerialBuffer(self):
        self.ReadSerialBuffer()

        if len(self.incommingSerialBuffer_) > self.MINIMUM_FROMARDUINO_PACKET_LENGTH:
            if not self.CheckUartHeader():
                self.incommingSerialBuffer_.pop(0)
            i2cPacketLength = self.incommingSerialBuffer_[self.I2C_TOPC_NBYTES]  # should be 32 long

            if len(self.incommingSerialBuffer_) > (i2cPacketLength + self.MINIMUM_FROMARDUINO_PACKET_LENGTH):
                if self.CheckUartFooter((i2cPacketLength + self.MINIMUM_FROMARDUINO_PACKET_LENGTH)):
                    toReturn = self.incommingSerialBuffer_[
                               0:(i2cPacketLength + self.MINIMUM_FROMARDUINO_PACKET_LENGTH + 1)]
                    self.incommingSerialBuffer_.clear()
                    return toReturn
                else:
                    self.incommingSerialBuffer_.clear()
                    return None
        return None

    def ReadSerialBuffer(self):
        if self.serialPort_.inWaiting():
            self.incommingSerialBuffer_ = bytearray(self.serialPort_.read(self.serialPort_.inWaiting()))
        # while self.serialPort_.inWaiting():
        #     print("within while loop",self.serialPort_.inWaiting())
        #     print("self.serialPort_.read(1)", self.serialPort_.read(48))
        #     self.incommingSerialBuffer_.append(self.serialPort_.read(1)[0])
        # print("Finsihed reading incomming SerialBuffer_",self.incommingSerialBuffer_)

    def CheckUartHeader(self):
        i = 0
        while i < 4:
            if (self.incommingSerialBuffer_[i] != 255) and (self.incommingSerialBuffer_[i] != 170):
                return False
            i += 1
        if self.incommingSerialBuffer_[0] == 170:
            self.isUSB = True
        return True

    def CheckUartFooter(self, endOfPacket):
        i = 0
        while i < 4:
            if self.incommingSerialBuffer_[(endOfPacket - i)] != 254:
                return False
            i += 1
        return True

    def WriteToCalibrationRegister(self, toSend, location, i2CAddress):
        if len(toSend) > 28:
            print("Trying to write a packet that is too large")
            return False
        if (len(toSend) + location) > 1024:
            print("Trying to write into read only region")
            return False
        cmdToArduino = serial_command.GenerateWriteCommand(i2CAddress, self.cmdItr_, location, toSend)
        self.serialPort_.write(cmdToArduino)
        acknowledged = False
        attempts = 5
        time.sleep(0.01)
        while (False == acknowledged) and (attempts > 0):
            cmdFromArduino = self.ProcessSerialBuffer()
            if cmdFromArduino is not None:
                if cmdToArduino[self.I2C_ID_BYTE] == cmdFromArduino[self.I2C_ID_BYTE]:
                    acknowledged = True
                    return True
                else:
                    print("Comms Error - Failed Acknoledge Write Command")
                    acknowledged = True
                    return False
            else:
                time.sleep(0.01)
                attempts -= 1
        return False
