# Singletact
This is a pythonized version of the .Net code provided by  [Singletact](https://github.com/SingleTact/NETInterface/blob/master/SingleTactLibrary/ArduinoSingleTactDriver.cs) to work with the arduino singletact on linux. This package works with the Single [USB version](https://www.singletact.com/wp-content/uploads/SingleTact_USB_QuickStartGuideV1.7.pdf). The package opens a serial port initiates the sensor and publishes a single float as a ROS message. 

## Installation
Relies on pyserial
```
pip3 install pyserial
```

## Running
Plug the USB sensor in and run
```
roslaunch singletact singletact.launch 'port_name':=/dev/ttyACM0
```
Assuming arduino is connected to /dev/ttyACM0, if you launch without a port_name parameter, all serial ports will be checked for an arduino connection. 

## ToDo
 * This publishes a single float value with no context, the message should have a sensor name, timestamp
 * This is tested for one sensor, for multiple sensors, the same node can be launched but it may be easier to modify the file to vectorize everything
