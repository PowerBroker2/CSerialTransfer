# CSerialTransfer
Linux Serial Transfer Library adapted from PowerBrokers SerialTransfer as mirror library.

[https://github.com/PowerBroker2/SerialTransfer](https://github.com/PowerBroker2/SerialTransfer)

The Syntax is nearly identical to the arduino version, with a few exceptions:

  The initial constructor is
  
    typedef void (*functionPtr)()
    SerialTransfer::SerialTransfer(const char *device, int baudRate, functionPtr _callbacks = NULL, uint8_t _callbacksLen = 0) 
    
    With device usually being some version of /dev/ttyeACM0 on linux and accepted
    baudrates being : 9600, 19200, 38400, 115200
    
example:
  SerialTransfer::SerialTransfer myTransfer("/dev/ttyACM0", 19200, callbacks, callbacksLen);
  
  Also consideration needed when transfering objects as packing differs from 8bit to 32bit, 
  
  struct myobj{
    uint8_t num;
    int num;}
   The above sent from an arduino is 5 bytes, while sent from a 32 bit machine is 8 bytes.
   
  
  
