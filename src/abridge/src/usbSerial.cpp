#include "usbSerial.h"

using namespace std;

USBSerial::USBSerial() {

}

void USBSerial::openUSBPort(string device_path, int baud)
{
  memset(&io_struct, 0, sizeof (io_struct));
  io_struct.c_iflag = 0;
  io_struct.c_oflag = 0;
  io_struct.c_cflag = CS8 | CREAD | CLOCAL;
  io_struct.c_lflag = 0;
  io_struct.c_cc[VMIN] = 1;
  io_struct.c_cc[VTIME] = 5;

  usb_file_descriptor = open(device_path.c_str(), O_RDWR | O_NONBLOCK);
  if (usb_file_descriptor <= 0)
  {
    cout << "Opening USB0 FAILED " << usb_file_descriptor << endl;
    exit(1);
  }
  cfsetospeed(&io_struct, B115200);
  cfsetispeed(&io_struct, B115200);
  tcsetattr(usb_file_descriptor, TCSANOW, &io_struct);
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

