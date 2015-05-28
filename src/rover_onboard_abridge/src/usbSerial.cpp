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
 * 
 * Revision Log:
 *      
 */

#include "usbSerial.h"

using namespace std;

USBSerial::USBSerial() {

}

void USBSerial::openUSBPort(char name[], int baud) {
    memset(&ioStruct, 0, sizeof (ioStruct));
    ioStruct.c_iflag = 0;
    ioStruct.c_oflag = 0;
    ioStruct.c_cflag = CS8 | CREAD | CLOCAL;
    ioStruct.c_lflag = 0;
    ioStruct.c_cc[VMIN] = 1;
    ioStruct.c_cc[VTIME] = 5;

    usbFileDescriptor = open(name, O_RDWR | O_NONBLOCK);
    if (usbFileDescriptor <= 0) {
        cout << "Opening USB0 FAILED " << usbFileDescriptor << endl;
        exit(1);
    }
    cfsetospeed(&ioStruct, B115200);
    cfsetispeed(&ioStruct, B115200);
    tcsetattr(usbFileDescriptor, TCSANOW, &ioStruct);
}

void USBSerial::sendData(char data[]) {
    sprintf(dataOut, "%s", data);
    write(usbFileDescriptor, dataOut, sizeof (dataOut));
    memset(&dataOut, '\0', sizeof (dataOut));
}

string USBSerial::readData() {
    if (read(usbFileDescriptor, &serialDataIn, sizeof (serialDataIn)) > 0) {
        read(usbFileDescriptor, &serialDataIn, sizeof (serialDataIn));
    }
    string str(serialDataIn);
    tcflush(usbFileDescriptor, TCIOFLUSH);
    memset(&serialDataIn, '\0', sizeof (serialDataIn));
    return str;
}

void USBSerial::closeUSBPort() {
    close(usbFileDescriptor);
}

USBSerial::~USBSerial() {
    closeUSBPort();
}

