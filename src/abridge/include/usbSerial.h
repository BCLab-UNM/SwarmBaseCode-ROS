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
  
    void openUSBPort(string device_path, int baud);
    void sendData(char data[]);
    string readData();
    void closeUSBPort();

private:

    struct termios io_struct;
    int usb_file_descriptor;
    char serial_data_in[200];
    char data_out[16];

};

#endif	/* USBSERIAL_H */

