#include "usbSerial.h" // Where is this file in this directory?

using namespace std;

USBSerial::USBSerial() {

}

void USBSerial::openUSBPort(string device_path, int baud)
{
  memset(&ioStruct, 0, sizeof (ioStruct));
  ioStruct.c_iflag = 0;
  ioStruct.c_oflag = 0;
  ioStruct.c_cflag = CS8 | CREAD | CLOCAL;
  ioStruct.c_lflag = 0;
  ioStruct.c_cc[VMIN] = 1;
  ioStruct.c_cc[VTIME] = 5;

  usb_file_descriptor = open(device_path.c_str(), O_RDWR | O_NONBLOCK);
  if (usb_file_descriptor <= 0)
  {
    cout << "Opening USB0 FAILED " << usb_file_descriptor << endl;
    exit(1);
  }
  cfsetospeed(&ioStruct, B115200);
  cfsetispeed(&ioStruct, B115200);
  tcsetattr(usb_file_descriptor, TCSANOW, &ioStruct);
}

void USBSerial::sendData(char data[])
{
  sprintf(data_out, "%s", data);
  write(usb_file_descriptor, data_out, sizeof (data_out));
  memset(&data_out, '\0', sizeof (data_out));
}

string USBSerial::readData()
{
  if (read(usb_file_descriptor, &serial_data_in, sizeof (serial_data_in)) > 0)
  {
    read(usb_file_descriptor, &serial_data_in, sizeof (serial_data_in));
  }
  string str(serial_data_in);
  tcflush(usb_file_descriptor, TCIOFLUSH);
  memset(&serial_data_in, '\0', sizeof (serial_data_in));
  return str;
}

void USBSerial::closeUSBPort()
{
  close(usb_file_descriptor);
}

USBSerial::~USBSerial()
{
  closeUSBPort();
}

