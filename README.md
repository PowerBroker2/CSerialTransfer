# CSerialTransfer
C++ Linux library for serial communication with Arduinos using the SerialTransfer protocol.

Tried to keep syntax as close as possible with the exception of the constructor as no debugging serial required

SerialTransfer::SerialTransfer(device, baudrate, callbackArr, callbackArrLength)

example:
SerialTransfer::SerialTransfer("/dev/ttyACM0", 19200, callbacks, callbacksLen)

