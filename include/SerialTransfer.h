#pragma once

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <ctime>
#include <math.h>
#include <string.h>
#include <iostream>
#include <stdexcept>



namespace SerialTransfer
{
    typedef void (*functionPtr)();

    class PacketCRC
    {
        public: // <<---------------------------------------//public
            uint8_t poly = 0;

            
            PacketCRC(const uint8_t& polynomial = 0x9B, const uint8_t& crcLen = 8)
            {
                poly      = polynomial;
                crcLen_   = crcLen;
                tableLen_ = pow(2, crcLen);
                csTable   = new uint8_t[tableLen_];

                generateTable();
            }

            void generateTable()
            {
                for (uint16_t i = 0; i < tableLen_; ++i)
                {
                    int curr = i;

                    for (int j = 0; j < 8; ++j)
                    {
                        if ((curr & 0x80) != 0)
                            curr = (curr << 1) ^ (int)poly;
                        else
                            curr <<= 1;
                    }

                    csTable[i] = (uint8_t)curr;
                }
            }

            void printTable()
	        {
                for (uint16_t i = 0; i < tableLen_; i++)
                {
                    std::cout << std::hex << csTable[i];
                    if ((i + 1) % 16)
                        std::cout << " ";
                    else
                        std::cout << std::endl;
                }
            }


            uint8_t calculate(const uint8_t& val)
            {
                if (val < tableLen_)
                    return csTable[val];
                return 0;
            }

            uint8_t calculate(uint8_t arr[], uint8_t len)
            {
                uint8_t crc = 0;
                for (uint16_t i = 0; i < len; i++)
                    crc = csTable[crc ^ arr[i]];

                return crc;
            }


        private: // <<---------------------------------------//private
            uint16_t tableLen_;
            uint8_t  crcLen_;
            uint8_t* csTable;
    };

        /*
    01111110 00000000 11111111 00000000 00000000 00000000 ... 00000000 10000001
    |      | |      | |      | |      | |      | |      | | | |      | |______|__Stop byte
    |      | |      | |      | |      | |      | |      | | | |______|___________8-bit CRC
    |      | |      | |      | |      | |      | |      | |_|____________________Rest of payload
    |      | |      | |      | |      | |      | |______|________________________2nd payload byte
    |      | |      | |      | |      | |______|_________________________________1st payload byte
    |      | |      | |      | |______|__________________________________________# of payload bytes
    |      | |      | |______|___________________________________________________COBS Overhead byte
    |      | |______|____________________________________________________________Packet ID (0 by default)
    |______|_____________________________________________________________________Start byte (constant)
    */


    

    class Packet
    {
        public: // <<---------------------------------------//public

           static constexpr uint8_t CONTINUE = 3;
           static constexpr uint8_t NEW_DATA = 2;
           static constexpr uint8_t NO_DATA = 1;
           static constexpr uint8_t CRC_ERROR = 0;
           static constexpr uint8_t PAYLOAD_ERROR      = -1;
           static constexpr uint8_t STOP_BYTE_ERROR    = -2;
           static constexpr uint8_t STALE_PACKET_ERROR = -3;
           static constexpr uint8_t START_BYTE = 0x7E;
           static constexpr uint8_t STOP_BYTE  = 0x81;
           static constexpr uint8_t PREAMBLE_SIZE   = 4;
           static constexpr uint8_t POSTAMBLE_SIZE  = 2;
           static constexpr uint8_t MAX_PACKET_SIZE = 254; // Maximum allowed payload bytes per packet
           static constexpr uint8_t DEFAULT_TIMEOUT = 50;

            uint8_t txBuff[MAX_PACKET_SIZE];
            uint8_t rxBuff[MAX_PACKET_SIZE];
            uint8_t preamble[PREAMBLE_SIZE]   = {START_BYTE, 0, 0, 0};
            uint8_t postamble[POSTAMBLE_SIZE] = {0, STOP_BYTE};

            uint8_t bytesRead = 0;
            int8_t  status    = 0;

            Packet(functionPtr* _callbacks, uint8_t _callbacksLen)
            {
                if (_callbacks && !_callbacksLen)
                    throw std::runtime_error("Number of Callbacks not entered");
                callbacksLen = _callbacksLen;
                callbacks = _callbacks;
            }
            
            /*
            uint8_t Packet::constructPacket(const uint16_t& messageLen, const uint8_t& packetID)
            Description:
            ------------
            * Calculate, format, and insert the packet protocol metadata into the packet transmit
            buffer
            Inputs:
            -------
            * const uint16_t& messageLen - Number of values in txBuff
            to send as the payload in the next packet
            * const uint8_t& packetID - The packet 8-bit identifier
            Return:
            -------
            * uint8_t - Number of payload bytes included in packet
            */
            uint8_t constructPacket(const uint16_t& messageLen, const uint8_t& packetID = 0)
            {
                if (messageLen > MAX_PACKET_SIZE)
                {
                    calcOverhead(txBuff, MAX_PACKET_SIZE);
                    stuffPacket(txBuff, MAX_PACKET_SIZE);
                    uint8_t crcVal = crc.calculate(txBuff, MAX_PACKET_SIZE);

                    preamble[1] = packetID;
                    preamble[2] = overheadByte;
                    preamble[3] = MAX_PACKET_SIZE;

                    postamble[0] = crcVal;

                    return MAX_PACKET_SIZE;
                }
                else
                {
                    calcOverhead(txBuff, (uint8_t)messageLen);
                    stuffPacket(txBuff, (uint8_t)messageLen);
                    uint8_t crcVal = crc.calculate(txBuff, (uint8_t)messageLen);

                    preamble[1] = packetID;
                    preamble[2] = overheadByte;
                    preamble[3] = messageLen;

                    postamble[0] = crcVal;

                    return (uint8_t)messageLen;
                }
            };
            /*
            uint8_t Packet::parse(const uint8_t& recChar, const bool& valid)
            Description:
            ------------
            * Parses incoming serial data, analyzes packet contents,
            and reports errors/successful packet reception. Executes
            callback functions for parsed packets whos ID has a
            corresponding callback function set via
            "void Packet::begin(const configST configs)"
            Inputs:
            -------
            * const uint8_t& recChar - Next char to parse in the stream
            * const bool& valid - Set if stream is "available()" and clear if not
            Return:
            -------
            * uint8_t - Num bytes in RX buffer
            */

            uint8_t parse(const uint8_t& recChar, const bool& valid)
            {
                bool packet_fresh = (packetStart == 0) || (((std::clock() - packetStart) / 1000) < 50);

                if(!packet_fresh) //packet is stale, start over.
                {
                    
                    std::cout << "ERROR: STALE PACKET" << std::endl;
                    bytesRead   = 0;
                    state       = find_start_byte;
                    status      = STALE_PACKET_ERROR;
                    packetStart = 0;

                    return bytesRead;
                }

                if (valid)
                {
                    switch (state)
                    {
                        case find_start_byte: /////////////////////////////////////////
                        {
                            if (recChar == START_BYTE)
                            {
                                state       = find_id_byte;
                                packetStart = std::clock();	//start the timer
                            }

                            break;
                        }

                        case find_id_byte: ////////////////////////////////////////////
                        {
                            idByte = recChar;
                            state  = find_overhead_byte;
                            break;
                        }

                        case find_overhead_byte: //////////////////////////////////////
                        {
                            recOverheadByte = recChar;
                            state           = find_payload_len;
                            break;
                        }

                        case find_payload_len: ////////////////////////////////////////
                        {
                            if ((recChar > 0) && (recChar <= MAX_PACKET_SIZE))
                            {
                                bytesToRec = recChar;
                                payIndex   = 0;
                                state      = find_payload;
                            }
                            else
                            {
                                bytesRead = 0;
                                state     = find_start_byte;
                                status    = PAYLOAD_ERROR;

                                std::cout << "ERROR: PAYLOAD_ERROR" << std::endl;

                                reset();
                                return bytesRead;
                            }
                            break;
                        }

                        case find_payload: ////////////////////////////////////////////
                        {
                            if (payIndex < bytesToRec)
                            {
                                rxBuff[payIndex] = recChar;
                                payIndex++;

                                if (payIndex == bytesToRec)
                                    state    = find_crc;
                            }
                            break;
                        }

                        case find_crc: ///////////////////////////////////////////
                        {
                            uint8_t calcCrc = crc.calculate(rxBuff, bytesToRec);

                            if (calcCrc == recChar)
                                state = find_end_byte;
                            else
                            {
                                bytesRead = 0;
                                state     = find_start_byte;
                                status    = CRC_ERROR;

                                std::cout << "ERROR: CRC_ERROR" << std::endl;

                                reset();
                                return bytesRead;
                            }

                            break;
                        }

                        case find_end_byte: ///////////////////////////////////////////
                        {
                            state = find_start_byte;

                            if (recChar == STOP_BYTE)
                            {
                                unpackPacket(rxBuff);
                                bytesRead = bytesToRec;
                                status    = NEW_DATA;

                                if (callbacks)
                                {
                                    if (idByte < callbacksLen)
                                        callbacks[idByte]();
                                    else
                                    {
                                        std::cout << "ERROR: No callback available for packet ID " << idByte << std::endl;
                                    }
                                }
                                packetStart = 0;	// reset the timer
                                return bytesToRec;
                            }

                            bytesRead = 0;
                            status    = STOP_BYTE_ERROR;
                            std::cout << "ERROR: STOP_BYTE_ERROR" << std::endl;
                            reset();
                            return bytesRead;
                            break;
                        }

                        default:
                        {
                            
                            std::cout << "ERROR: Undefined state " << state << std::endl;
                            reset();
                            bytesRead = 0;
                            state     = find_start_byte;
                            break;
                        }
                    }
                }
                else
                {
                    bytesRead = 0;
                    status    = NO_DATA;
                    return bytesRead;
                }

                bytesRead = 0;
                status    = CONTINUE;
                return bytesRead;
            }

            /*
            uint8_t Packet::currentPacketID()
            Description:
            ------------
            * Returns the ID of the last parsed packet
            Inputs:
            -------
            * void
            Return:
            -------
            * uint8_t - ID of the last parsed packet
            */
            uint8_t currentPacketID()
            {
                return idByte;
            }

            /*
            void Packet::reset()
            Description:
            ------------
            * Clears out the tx, and rx buffers, plus resets
            the "bytes read" variable, finite state machine, etc
            Inputs:
            -------
            * void
            Return:
            -------
            * void
            */
            void reset()
            {
                memset(txBuff, 0, sizeof(txBuff));
                memset(rxBuff, 0, sizeof(rxBuff));

                bytesRead   = 0;
                status      = CONTINUE;
                packetStart = 0;
            }


            /*
            uint16_t Packet::txObj(const T &val, const uint16_t &index=0, const uint16_t &len=sizeof(T))
            Description:
            ------------
            * Stuffs "len" number of bytes of an arbitrary object (byte, int,
            float, double, struct, etc...) into the transmit buffer (txBuff)
            starting at the index as specified by the argument "index"

            Inputs:
            -------
            * const T &val - Pointer to the object to be copied to the
            transmit buffer (txBuff)
            * const uint16_t &index - Starting index of the object within the
            transmit buffer (txBuff)
            * const uint16_t &len - Number of bytes of the object "val" to transmit

            Return:
            -------
            * uint16_t maxIndex - uint16_t maxIndex - Index of the transmit buffer (txBuff) that directly follows the bytes processed
            by the calling of this member function
            */
            template <typename T>
            uint16_t txObj(const T& val, const uint16_t& index = 0, const uint16_t& len = sizeof(T))
            {
                uint8_t* ptr = (uint8_t*)&val;
                uint16_t maxIndex;

                if ((len + index) > MAX_PACKET_SIZE)
                    maxIndex = MAX_PACKET_SIZE;
                else
                    maxIndex = len + index;

                for (uint16_t i = index; i < maxIndex; i++)
                {
                    txBuff[i] = *ptr;
                    ptr++;
                }

                return maxIndex;
            }


            /*
            uint16_t Packet::rxObj(const T &val, const uint16_t &index=0, const uint16_t &len=sizeof(T))
            Description:
            ------------
            * Reads "len" number of bytes from the receive buffer (rxBuff)
            starting at the index as specified by the argument "index"
            into an arbitrary object (byte, int, float, double, struct, etc...)

            Inputs:
            -------
            * const T &val - Pointer to the object to be copied into from the
            receive buffer (rxBuff)
            * const uint16_t &index - Starting index of the object within the
            receive buffer (rxBuff)
            * const uint16_t &len - Number of bytes in the object "val" received

            Return:
            -------
            * uint16_t maxIndex - Index of the receive buffer (rxBuff) that directly follows the bytes processed
            by the calling of this member function
            */
            template <typename T>
            uint16_t rxObj(const T& val, const uint16_t& index = 0, const uint16_t& len = sizeof(T))
            {
                uint8_t* ptr = (uint8_t*)&val;
                uint16_t maxIndex;

                if ((len + index) > MAX_PACKET_SIZE)
                    maxIndex = MAX_PACKET_SIZE;
                else
                    maxIndex = len + index;

                for (uint16_t i = index; i < maxIndex; i++)
                {
                    *ptr = rxBuff[i];
                    ptr++;
                }

                return maxIndex;
            }


        private: // <<---------------------------------------//private
            enum fsm
            {
                find_start_byte,
                find_id_byte,
                find_overhead_byte,
                find_payload_len,
                find_payload,
                find_crc,
                find_end_byte
            };

            fsm state = find_start_byte;

            PacketCRC crc;
            functionPtr* callbacks;
            uint8_t callbacksLen;

   
            uint8_t bytesToRec      = 0;
            uint8_t payIndex        = 0;
            uint8_t idByte          = 0;
            uint8_t overheadByte    = 0;
            uint8_t recOverheadByte = 0;

            uint32_t packetStart    = 0;
            uint32_t timeout;


            /*
            void Packet::calcOverhead(uint8_t arr[], const uint8_t &len)
            Description:
            ------------
            * Calculates the COBS (Consistent Overhead Stuffing) Overhead
            byte and stores it in the class's overheadByte variable. This
            variable holds the byte position (within the payload) of the
            first payload byte equal to that of START_BYTE
            Inputs:
            -------
            * uint8_t arr[] - Array of values the overhead is to be calculated
            over
            * const uint8_t &len - Number of elements in arr[]
            Return:
            -------
            * void
            */
            void calcOverhead(uint8_t arr[], const uint8_t& len)
            {
                overheadByte = 0xFF;

                for (uint8_t i = 0; i < len; i++)
                {
                    if (arr[i] == START_BYTE)
                    {
                        overheadByte = i;
                        break;
                    }
                }
            };


            /*
            int16_t Packet::findLast(uint8_t arr[], const uint8_t &len)
            Description:
            ------------
            * Finds last instance of the value START_BYTE within the given
            packet array
            Inputs:
            -------
            * uint8_t arr[] - Packet array
            * const uint8_t &len - Number of elements in arr[]
            Return:
            -------
            * int16_t - Index of last instance of the value START_BYTE within the given
            packet array
            */
            int16_t findLast(uint8_t arr[], const uint8_t& len)
            {
                for (uint8_t i = (len - 1); i != 0xFF; i--)
                    if (arr[i] == START_BYTE)
                        return i;

                return -1;
            };


            /*
            void Packet::stuffPacket(uint8_t arr[], const uint8_t &len)
            Description:
            ------------
            * Enforces the COBS (Consistent Overhead Stuffing) ruleset across
            all bytes in the packet against the value of START_BYTE
            Inputs:
            -------
            * uint8_t arr[] - Array of values to stuff
            * const uint8_t &len - Number of elements in arr[]
            Return:
            -------
            * void
            */
            void stuffPacket(uint8_t arr[], const uint8_t& len)
            {
                int16_t refByte = findLast(arr, len);

                if (refByte != -1)
                {
                    for (uint8_t i = (len - 1); i != 0xFF; i--)
                    {
                        if (arr[i] == START_BYTE)
                        {
                            arr[i]  = refByte - i;
                            refByte = i;
                        }
                    }
                }
            };


            /*
            void Packet::unpackPacket(uint8_t arr[], const uint8_t &len)
            Description:
            ------------
            * Unpacks all COBS-stuffed bytes within the array
            Inputs:
            -------
            * uint8_t arr[] - Array of values to unpack
            * const uint8_t &len - Number of elements in arr[]
            Return:
            -------
            * void
            */
            void unpackPacket(uint8_t arr[])
            {
                uint8_t testIndex = recOverheadByte;
                uint8_t delta     = 0;

                if (testIndex <= MAX_PACKET_SIZE)
                {
                    while (arr[testIndex])
                    {
                        delta          = arr[testIndex];
                        arr[testIndex] = START_BYTE;
                        testIndex += delta;
                    }
                    arr[testIndex] = START_BYTE;
                }
            }

        };

    class SerialTransfer
    {
        public: // <<---------------------------------------//public
            Packet  packet;
            uint8_t bytesRead = 0;
            int8_t  status    = 0;

            SerialTransfer(const char* device, int baudRate, functionPtr* _callbacks = NULL, uint8_t _callbacksLen = 0) : packet(_callbacks, _callbacksLen)
            {       
                serial_port = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
                
                if (serial_port < 0)
                    throw std::runtime_error("Device unavailable!");

                if (tcflush(serial_port, TCIOFLUSH))
                    std::cout << "WARNING: tcflush failed." << std::endl;

                if (tcgetattr(serial_port, &tty))
                    std::cout << "WARNING: unable to get device info for " << device << std::endl;
                tcgetattr(serial_port, &ttyb);

                tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
                tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
                tty.c_cflag &= ~CSIZE; // Clear all the size bits
                tty.c_cflag |= CS8; // 8 bits per byte
                tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
                tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
                tty.c_lflag &= ~ICANON;
                tty.c_lflag &= ~ECHO; // Disable echo
                tty.c_lflag &= ~ECHOE; // Disable erasure
                tty.c_lflag &= ~ECHONL; // Disable new-line echo
                tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
                tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
                tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
                tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
                tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

                switch (baudRate)
                {
                    case 9600:   cfsetospeed(&tty, B9600);   break;
                    case 19200:  cfsetospeed(&tty, B19200);  break;
                    case 38400:  cfsetospeed(&tty, B38400);  break;
                    case 115200: cfsetospeed(&tty, B115200); break;
                    default:
                        std::cout << "WARNING: baud rate: " << baudRate << "is not supported, using 9600." << std::endl;
                        cfsetospeed(&tty, B9600);
                        break;
                }
                //do not block or time read, allow parse to handle timeouts
                tty.c_cc[VTIME] = 0;
                tty.c_cc[VMIN] = 0;

                if (tcsetattr(serial_port, TCSANOW, &tty))
                {
                    close(serial_port);
                    throw std::runtime_error("Unable to set serial port tty!");  
                }
                std::cout << "Successfully created serial connection!" << std::endl;        
            };

            ~SerialTransfer()
            {
                tcsetattr(serial_port, TCSANOW, &ttyb);
                close(serial_port);        
            }

            /*
            uint8_t SerialTransfer::sendData(const uint16_t &messageLen, const uint8_t packetID)
            Description:
            ------------
            * Send a specified number of bytes in packetized form
            Inputs:
            -------
            * const uint16_t &messageLen - Number of values in txBuff
            to send as the payload in the next packet
            * const uint8_t packetID - The packet 8-bit identifier
            Return:
            -------
            * uint8_t numBytesIncl - Number of payload bytes included in packet
            */
            uint8_t sendData(const uint16_t& messageLen, const uint8_t packetID = 0)
            {
                uint8_t numBytesIncl;

                numBytesIncl = packet.constructPacket(messageLen, packetID);
                write(serial_port, packet.preamble, sizeof(packet.preamble));
                write(serial_port, packet.txBuff, numBytesIncl);
                write(serial_port, packet.postamble, sizeof(packet.postamble));

                return numBytesIncl;
            };


            /*
            uint8_t SerialTransfer::available()
            Description:
            ------------
            * Parses incoming serial data, analyzes packet contents,
            and reports errors/successful packet reception
            Inputs:
            -------
            * void
            Return:
            -------
            * uint8_t bytesRead - Num bytes in RX buffer
            */
            uint8_t available()
            {
                bool    valid   = false;
                uint8_t recChar = 0xFF;
                int     fion = 0;
                ioctl(serial_port, FIONREAD, &fion);
                if (fion)
                    {
                    
                        valid = true;
                        for (int i = 0; i < fion; i++)
                        {
                            read(serial_port, &recChar, sizeof(recChar));
                            bytesRead = packet.parse(recChar, valid);
                            status    = packet.status;

                            if (status != packet.CONTINUE)
                            {
                                if (status < 0)
                                    reset();

                                break;
                            }
                        }
                    }
                    else
                    {
                        bytesRead = packet.parse(recChar, valid);
                        status    = packet.status;

                        if (status < 0)
                            reset();
                    }

                return bytesRead;
            };


            /*
            bool SerialTransfer::tick()
            Description:
            ------------
            * Checks to see if any packets have been fully parsed. This
            is basically a wrapper around the method "available()" and
            is used primarily in conjunction with callbacks
            Inputs:
            -------
            * void
            Return:
            -------
            * bool - Whether or not a full packet has been parsed
            */
            bool tick()
            {
                if (available())
                    return true;

                return false;
            };


            /*
            uint8_t SerialTransfer::currentPacketID()
            Description:
            ------------
            * Returns the ID of the last parsed packet
            Inputs:
            -------
            * void
            Return:
            -------
            * uint8_t - ID of the last parsed packet
            */
            uint8_t currentPacketID()
            {
                return packet.currentPacketID();
            };


            /*
            void SerialTransfer::reset()
            Description:
            ------------
            * Clears out the tx, and rx buffers, plus resets
            the "bytes read" variable, finite state machine, etc
            Inputs:
            -------
            * void
            Return:
            -------
            * void
            */
            void reset()
            {
                packet.reset();
                status = packet.status;
            };


            /*
            uint16_t SerialTransfer::txObj(const T &val, const uint16_t &index=0, const uint16_t &len=sizeof(T))
            Description:
            ------------
            * Stuffs "len" number of bytes of an arbitrary object (byte, int,
            float, double, struct, etc...) into the transmit buffer (txBuff)
            starting at the index as specified by the argument "index"
            Inputs:
            -------
            * const T &val - Pointer to the object to be copied to the
            transmit buffer (txBuff)
            * const uint16_t &index - Starting index of the object within the
            transmit buffer (txBuff)
            * const uint16_t &len - Number of bytes of the object "val" to transmit
            Return:
            -------
            * uint16_t maxIndex - uint16_t maxIndex - Index of the transmit buffer (txBuff) that directly follows the bytes processed
            by the calling of this member function
            */
            template <typename T>
            uint16_t txObj(const T& val, const uint16_t& index = 0, const uint16_t& len = sizeof(T))
            {
                return packet.txObj(val, index, len);
            };


            /*
            uint16_t SerialTransfer::rxObj(const T &val, const uint16_t &index=0, const uint16_t &len=sizeof(T))
            Description:
            ------------
            * Reads "len" number of bytes from the receive buffer (rxBuff)
            starting at the index as specified by the argument "index"
            into an arbitrary object (byte, int, float, double, struct, etc...)
            Inputs:
            -------
            * const T &val - Pointer to the object to be copied into from the
            receive buffer (rxBuff)
            * const uint16_t &index - Starting index of the object within the
            receive buffer (rxBuff)
            * const uint16_t &len - Number of bytes in the object "val" received
            Return:
            -------
            * uint16_t maxIndex - Index of the receive buffer (rxBuff) that directly follows the bytes processed
            by the calling of this member function
            */
            template <typename T>
            uint16_t rxObj(const T& val, const uint16_t& index = 0, const uint16_t& len = sizeof(T))
            {
                return packet.rxObj(val, index, len);
            };


            /*
            uint8_t SerialTransfer::sendDatum(const T &val, const uint16_t &len=sizeof(T))
            Description:
            ------------
            * Stuffs "len" number of bytes of an arbitrary object (byte, int,
            float, double, struct, etc...) into the transmit buffer (txBuff)
            starting at the index as specified by the argument "index" and
            automatically transmits the bytes in an individual packet
            Inputs:
            -------
            * const T &val - Pointer to the object to be copied to the
            transmit buffer (txBuff)
            * const uint16_t &len - Number of bytes of the object "val" to transmit
            Return:
            -------
            * uint8_t - Number of payload bytes included in packet
            */
            template <typename T>
            uint8_t sendDatum(const T& val, const uint16_t& len = sizeof(T))
            {
                return sendData(packet.txObj(val, 0, len));
            };


        private: // <<---------------------------------------//private
            int serial_port;
            uint32_t timeout;
            struct termios tty;
            struct termios ttyb;
        };

}
