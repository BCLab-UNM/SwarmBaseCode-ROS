#ifndef USBSERIAL_H
#define	USBSERIAL_H

#include <cstdlib>
#include <string>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <unistd.h>  
#include <fcntl.h>   
#include <termios.h> 

using namespace std;

class USBSerial {
public:
    
    USBSerial();
    virtual ~USBSerial();
  
    void openUSBPort(string devicePath, int baud);
    void sendData(char data[]);
    string readData();
    void closeUSBPort();

private:

    struct termios ioStruct;
    int usbFileDescriptor;
    char serialDataIn[200];
    char dataOut[16];

};

#endif	/* USBSERIAL_H */

