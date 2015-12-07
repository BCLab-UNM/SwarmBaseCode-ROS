#include <cstdlib>
#include "usbCamera.h"

using namespace std;

int main(int argc, char* argv[]) {

    char host[128];
    gethostname(host, sizeof (host));
    string hostname(host);
    string publishedName;

    ros::init(argc, argv, (hostname + "_CAMERA"));

    ros::NodeHandle tNH;

    int cameraIndex = 0;

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  Camera module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    USBCamera usbCam1(10, cameraIndex, publishedName);

    while (ros::ok()) {
        ros::spin();
    }

    return EXIT_SUCCESS;
}
