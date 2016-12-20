#include <signal.h>
#include "sbridge.h"

using namespace std;

void sigintEventHandler(int signal);

int main(int argc, char **argv) {
    sleep(10);

    char host[128];
    gethostname(host, sizeof (host));
    string hostname(host);
    string publishedName;
    
    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  ABridge module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    ros::init(argc, argv, (hostname + "_SBRIDGE"), ros::init_options::NoSigintHandler);

    sbridge sb(publishedName);

    signal(SIGINT, sigintEventHandler);

    ros::spin();

	return EXIT_SUCCESS;
}

void sigintEventHandler(int signal) {
    ros::shutdown();
}

