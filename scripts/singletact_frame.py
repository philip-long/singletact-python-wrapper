class SingleTactFrame:
    def __init__(self, sensorData, timeStamp):
        self.nSensors_ = len(sensorData)
        self.sensorDataRaw_ = sensorData
        self.timeStamp_ = timeStamp
        self.sensorData_ = []
        for i in sensorData:
            self.sensorData_.append(i - 0xFF)
