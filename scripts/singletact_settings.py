class SingleTactSettings:
    def __init__(self):
        self.INDEX_SMBUSADDRESS = 0
        self.INDEX_SERIAL_NUMBER_MSB = 1
        self.INDEX_SERIAL_NUMBER_LSB = 2
        self.INDEX_SN_MSB = 3
        self.INDEX_SN_LSB = 4
        self.INDEX_Accumulator = 5
        self.INDEX_REFERENCE_GAIN = 6
        self.INDEX_FIRMWARE_REVISION = 7
        self.INDEX_DISCHARGE_TIMER = 8
        self.INDEX_OUTPUT_CURRENT = 9
        self.INDEX_SCALING_MSB = 10
        self.INDEX_SCALING_LSB = 11
        self.INDEX_NUMBER_ELEMENTS = 12
        self.INDEX_CALIBRATION = 13
        self.INDEX_INDEX_OF_SCANLIST = 15
        self.INDEX_OF_BASELINES = 41
        self.settingsRaw_=[]

    def getNumberElements(self):
        return self.settingsRaw_[self.INDEX_NUMBER_ELEMENTS]

    def NumberElements(self, value):
        self.settingsRaw_[self.INDEX_NUMBER_ELEMENTS] = value

    def getSettingsRaw(self):
        return self.settingsRaw_

    def SettingsRaw(self, value):
        self.settingsRaw_ = value

    def I2CAddress(self):
        return self.settingsRaw_[self.INDEX_SMBUSADDRESS]

    def I2CAddress(self, value):
        self.settingsRaw_[self.INDEX_SMBUSADDRESS] = value

    def getSerialNumberMsb(self):
        return (self.settingsRaw_[self.INDEX_SERIAL_NUMBER_MSB] * 256) + self.settingsRaw_[self.INDEX_SERIAL_NUMBER_LSB]

    def SerialNumberMsb(self, value):
        self.settingsRaw_[self.INDEX_SERIAL_NUMBER_MSB] = (value >> 8)
        self.settingsRaw_[self.INDEX_SERIAL_NUMBER_LSB] = (value & 255)

    def getFirmwareVersion(self):
        return  self.settingsRaw_[ self.INDEX_FIRMWARE_REVISION]

    def FirmwareVersion(self, value):
        self.settingsRaw_[ self.INDEX_FIRMWARE_REVISION] = value

    def getAccumulator(self):
        return  self.settingsRaw_[ self.INDEX_Accumulator]

    def Accumulator(self, value):
        self.settingsRaw_[ self.INDEX_Accumulator] = value

    def getReferenceGain(self):
        return  self.settingsRaw_[ self.INDEX_REFERENCE_GAIN]

    def ReferenceGain(self, value):
        self.settingsRaw_[ self.INDEX_REFERENCE_GAIN] = value

    def getDischargeTimer(self):
        return  self.settingsRaw_[ self.INDEX_DISCHARGE_TIMER]


    def DischargeTimer(self, value):
        self.settingsRaw_[ self.INDEX_DISCHARGE_TIMER] = value


    def getScaling(self):
        return ( self.settingsRaw_[ self.INDEX_SCALING_MSB] * 256) +  self.settingsRaw_[ self.INDEX_SCALING_LSB]

    def Scaling(self, value):
        self.settingsRaw_[self.INDEX_SCALING_MSB] = ((value >> 8))
        self.settingsRaw_[self.INDEX_SCALING_LSB] = ((value & 255))

    def getOutputCurrent(self):
        return self.settingsRaw_[self.INDEX_OUTPUT_CURRENT]

    def OutputCurrent(self, value):
        self.settingsRaw_[self.INDEX_OUTPUT_CURRENT] = value

    def getCalibrated(self):
        return self.settingsRaw_[self.INDEX_CALIBRATION]

    def Calibrated(self, value):
        self.settingsRaw_[self.INDEX_CALIBRATION] = value


    def getScanList(self):
        i = 0
        toReturn=bytearray(self.settingsRaw_[self.INDEX_NUMBER_ELEMENTS])
        while i < len(toReturn):
            toReturn[i] = self.settingsRaw_[(self.INDEX_INDEX_OF_SCANLIST + i)]
            i += 1
        return toReturn


    def ScanList(self, value):
        if len(value) > 25:
            print("Error: Too many elements Error")
            return False
        self.settingsRaw_[self.INDEX_NUMBER_ELEMENTS] = len(value)

        i = 0
        while i < len(value):
            self.settingsRaw_[(self.INDEX_INDEX_OF_SCANLIST + i)] = value[i]
            i += 1
        i = value.Length
        while i < 25:
            self.settingsRaw_[(self.INDEX_INDEX_OF_SCANLIST + i)] = 0
            i += 1

    def getBaselines(self):
        toReturn = []
        i = 0
        while i < len(toReturn):
            toReturn[i] = (self.settingsRaw_[(self.INDEX_OF_BASELINES + (2 * i))] << 8) \
                             + (self.settingsRaw_[((self.INDEX_OF_BASELINES + (2 * i)) + 1)])
            i += 1
        return toReturn

    def Baselines(self, value):
        if len(value) != self.NumberElements:
            print("Error: Does not match number of elements", "Error")
            return

        i = 0
        while i < len(value) * 2:
            self.settingsRaw_[(self.INDEX_OF_BASELINES + i)] = (value[i] >> 8)
            self.settingsRaw_[((self.INDEX_OF_BASELINES + i) + 1)] = (value[i])
            i += 2
        i = (len(value) * 2)
        while i < 50:
            self.settingsRaw_[(self.INDEX_OF_BASELINES + i)] = 0
            i += 1
