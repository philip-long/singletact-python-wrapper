import time
import singletact_frame
import singletact_settings
import arduino_single_tact
import rospy


class USBDevice:

    # parameterized constructor
    def __init__(self, port_name):
        self.port_name = port_name
        self.single_tact = SingleTact(port_name)
        self.lastTimestamp = 0.0
        rospy.loginfo("Singletact is on serial port %s", self.port_name)

    def setTimestamp(self, timestamp):
        self.lastTimestamp = timestamp


class SingleTact:

    def __init__(self, port_name):
        rospy.loginfo("Single tact library has started")
        self.param_location = 112
        self.Settings = singletact_settings.SingleTactSettings()
        self.lastFrame_ = None
        self.arduino = arduino_single_tact.ArduinoSingleTactDriver(port_name)
        self.startTime = 0
        self.i2cAddress_ = 0x04
        self.itr_ = 0
        self.PullSettingsFromHardware()
        self.isUSB = self.arduino.isUSB
        self.isFirst = True
        self.isConnected = False

        self.isCalibrated = self.Settings.getCalibrated()
        self.firmwareVersion = self.Settings.getFirmwareVersion()  # const int INDEX_FIRMWARE_REVISION = 7;

    def PushCalibrationToHardware(self, calibrationTable):
        i = 0
        while i < 32:
            PacketSize = 16
            toSend = bytearray(PacketSize)
            j = 0
            while j < (PacketSize / 2):
                toSend[(j * 2)] = (calibrationTable[((i * 8) + j)] >> 8)
                toSend[((j * 2) + 1)] = (calibrationTable[((i * 8) + j)] & 255)
                j += 1
            if not self.arduino.WriteToCalibrationRegister(toSend, i, self.arduino.i2cAddress_):
                rospy.logerr("Failed to write calibration")
                self.isCalibrated = False
            time.sleep(0.1)
            i += 1
        self.isCalibrated = True

    def PushSettingsToHardware(self):
        for i in range(2):
            PacketSize = 16
            toSend = bytearray(PacketSize)
            j = 0

            while j < PacketSize:
                toSend[j] = self.Settings.getSettingsRaw()[((i * PacketSize) + j)]
                j += 1

            if not self.arduino.WriteToMainRegister(toSend, (i * PacketSize), self.i2cAddress_):
                rospy.logerr("Failed to write settings")
            else:
                rospy.loginfo("Successfully wrote settings to main register")

            time.sleep(0.05)

    def PullSettingsFromHardware(self):
        rospy.loginfo("Pull Settings from hardware")
        settings_raw = bytearray(self.param_location)
        parameters_raw = bytearray(128 - self.param_location)

        for i in range(4):
            newByteData = self.arduino.ReadFromMainRegister(i * 32, 32, self.i2cAddress_)
            if newByteData is None:
                return False
            for j in range(32):
                if ((i * 32) + j) < self.param_location:
                    settings_raw[((i * 32) + j)] = newByteData[(j + self.arduino.TIMESTAMP_SIZE)]
                else:
                    parameters_raw[(j - 16)] = newByteData[(j + self.arduino.TIMESTAMP_SIZE)]

        self.Settings.SettingsRaw(settings_raw)
        time.sleep(1)
        return True

    def ReadSensorData(self):

        newByteData = self.arduino.ReadFromMainRegister(128, 6, self.i2cAddress_)

        if newByteData is not None:

            itr = (((newByteData[(0 + self.arduino.TIMESTAMP_SIZE)] << 8) + newByteData[
                (1 + self.arduino.TIMESTAMP_SIZE)]))

            if (self.itr_ == itr) and (self.lastFrame_ is not None):
                return self.lastFrame_

            itr_ = itr
            timeStampRaw = (newByteData[0] << 24) + (newByteData[1] << 16) + (newByteData[2] << 8) + newByteData[3]

            if self.isFirst:
                self.isFirst = False
                self.startTime = timeStampRaw
            timeStampRaw -= self.startTime
            timeStamp = (float(timeStampRaw) / 10000.0)
            sensorData = [0] * int((len(newByteData) - self.arduino.TIMESTAMP_SIZE - 4) / 2)


            #      //Do it manually to avoid confusion with Endianess - Windows is little, sensor is big.
            #                  find a more elegant solution!
            for i in range(len(sensorData)):
                sensorData[i] = (newByteData[(((2 * i) + 4) + self.arduino.TIMESTAMP_SIZE)] << 8) + \
                                newByteData[(((2 * i) + 5) + self.arduino.TIMESTAMP_SIZE)]

            toReturn = singletact_frame.SingleTactFrame(sensorData, timeStamp)
            self.lastFrame_ = singletact_frame.SingleTactFrame(sensorData, timeStamp)
            return toReturn
        else:
            return None

    def Tare(self):
        scaling = self.Settings.getScaling()
        if self.lastFrame_ is None:
            return False

        self.Settings.Baselines(self.lastFrame_.nSensors_)
        self.Settings.Scaling(100)

        self.PushSettingsToHardware()
        time.sleep(0.1)
        newFrame = self.ReadSensorData()
        newBaselines = []

        for i in range(len(newFrame.sensorDataRaw_)):
            newBaselines[i] = newFrame.sensorDataRaw_[i] - 255


        self.Settings.Baselines(newBaselines)
        self.Settings.Scaling(scaling)

        self.PushSettingsToHardware()
        return True
