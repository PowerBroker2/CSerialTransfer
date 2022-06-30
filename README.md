# CSerialTransfer
C++ Linux library for serial communication with Arduinos using the SerialTransfer protocol.

Tried to keep syntax as close as possible with the exception of the constructor as no debugging serial required

Also removed all globals and macros and placed in nsamespoace SerialTransfer

Constructor, no begin command required:
SerialTransfer::SerialTransfer(device, baudrate, callbackArr, callbackArrLength)

example:
  SerialTransfer::SerialTransfer myTransfer("/dev/ttyACM0", 19200, callbacks, callbacksLen);
  
  Code was well commented and used it in my project, thank you!
