#include <signal.h>
#include "sbridge.h"

using namespace std;

void sigintEventHandler(int signal);

int main(int argc, char **argv)
{
  sleep(10);

  char host[128];
  gethostname(host, sizeof (host));
  string hostname(host);
  string published_name;

  if (argc >= 2)
  {
      published_name = argv[1];
      cout << "Welcome to the world of tomorrow " << published_name << "!  ABridge module started." << endl;
  }
  else
  {
      published_name = hostname;
      cout << "No Name Selected. Default is: " << published_name << endl;
  }

  ros::init(argc, argv, (hostname + "_SBRIDGE"), ros::init_options::NoSigintHandler);

  sbridge sb(published_name);

  signal(SIGINT, sigintEventHandler);

  ros::spin();

	return EXIT_SUCCESS;
}

void sigintEventHandler(int signal)
{
  ros::shutdown();
}

