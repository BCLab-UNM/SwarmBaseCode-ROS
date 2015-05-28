/*
 * Author: Karl A. Stolleis
 * Maintainer: Karl A. Stolleis
 * Email: karl.a.stolleis@nasa.gov; kurt.leucht@nasa.gov
 * NASA Center: Kennedy Space Center
 * Mail Stop: NE-C1
 * 
 * Project Name: Swarmie Robotics NASA Center Innovation Fund
 * Principal Investigator: Cheryle Mako
 * Email: cheryle.l.mako@nasa.gov
 * 
 * Date Created: June 6, 2014
 * Safety Critical: NO
 * NASA Software Classification: D
 * 
 * This software is copyright the National Aeronautics and Space Administration (NASA)
 * and is distributed under the GNU LGPL license.  All rights reserved.
 * Permission to use, copy, modify and distribute this software is granted under
 * the LGPL and there is no implied warranty for this software.  This software is provided
 * "as is" and NASA or the authors are not responsible for indirect or direct damage
 * to any user of the software.  The authors and NASA are under no obligation to provide
 * maintenence, updates, support or modifications to the software.
 */

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
  
    void openUSBPort(char name[], int baud);
    void sendData(char data[]);
    string readData();
    void closeUSBPort();

private:

    struct termios ioStruct;
    int usbFileDescriptor;
    char serialDataIn[128];
    char dataOut[16];

};

#endif	/* USBSERIAL_H */

