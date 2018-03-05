#include "SearchController.h"
#include <angles/angles.h>

SearchController::SearchController()
{
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;
}

void SearchController::Reset() {
  result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork()
{
  result.type = waypoint;
  Point searchLocation;

  if(!targetFound) //If we haven't found a tag yet
  {

    if(first_waypoint)
    {
      first_waypoint = false;
      initialHeading = angles::normalize_angle_positive(currentLocation.theta);
    }

    if(obstSeen && obstacleCalls > 4)
    {
      searchLocation = clusterLocation;
      obstacleCalls = 0;
    }
    else if(!obstSeen)
    {
      distance = distance + 0.5;
      obstacleCalls = 0;
    }
    else
    {
      obstSeen = false;
    }

    searchLocation.x = currentLocation.x + (0.5 * cos(initialHeading));
    searchLocation.y = currentLocation.y + (0.5 * sin(initialHeading));

    cout << "Distance from center is going to be: " << distance << endl;

    //store clusterLocation in case rover finds a cluster
    clusterLocation.x = centerLocation.x + ((distance + cluster_offset)  * cos(initialHeading));
    clusterLocation.y = centerLocation.y + 0.8 + ((distance + cluster_offset) * sin(initialHeading));

    cout << "Cluster X:  " << clusterLocation.x << ", Cluster Y:  " << clusterLocation.y <<  endl;

    cout << "(SetCenterLocation)Center X:  " << centerLocation.x << ", Center Y:  " << centerLocation.y << endl;
  }
  else //If we have found a tag and are going back to finding other tags
  {
    if(!obstSeen && obstacleCalls > 0)
    {
      obstacleCalls = 0;
    }

    float clusterDistance = hypot(currentLocation.x - clusterLocation.x, currentLocation.y - clusterLocation.y);      //current distance from cluster and robot
    float distanceTolerance = 0.10;   //error allowed between cluster and robot

    cout << "Cluster Distance:  " << clusterDistance << endl;

    //if not within a specific distance of cluster, go to cluster
    if(clusterDistance > distanceTolerance)
    {
      searchLocation.x = clusterLocation.x;
      searchLocation.y = clusterLocation.y;

      cout << "driving back to cluster" << endl;
    }

    //if near cluster search around for more cubes...
    else
    {
      if(!obstSeen && obstacleCalls > 0)
      {
        obstacleCalls = 0;
      }

      //Search around cluster aimlessly
      searchLocation.theta = rng->gaussian(currentLocation.theta, M_PI);
      searchLocation.x = clusterLocation.x + (.5) * cos(searchLocation.theta);
      searchLocation.y = clusterLocation.y + (.5) * sin(searchLocation.theta);

      cout << "Searching around cluster for tags" << endl;
    }

  }

  //For Testing in SIM currently checking center location
  /*
  if(distance >= 10)
  {
    searchLocation = centerLocation;
    distance = 1;

    cout << "driving back to center";

  }
  */
  /*
  //select new position 50 cm from current location
  if (first_waypoint)
  {
    first_waypoint = false;
    searchLocation.theta = currentLocation.theta + M_PI;
    searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
    searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
  }
  else
  {
    //select new heading from Gaussian distribution around current heading
    searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
    searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
    searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
  }
*/
  result.wpts.waypoints.clear();
  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

  return result;
}

void SearchController::SetCenterLocation(Point centerLocation)
{
  if(targetFound)
  {
   // float cluster_diff_theta = atan2(centerLocation.y - clusterLocation.y, centerLocation.x - clusterLocation.x);

   // float offset = 2.5;

    //centerLocation.x = centerLocation.x + (offset * cos(cluster_diff_theta));
    //centerLocation.y = centerLocation.y + (offset * sin(cluster_diff_theta));
     // centerLocation.x -= 1;
  }

  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  
  if (!result.wpts.waypoints.empty())
  {
  //  clusterLocation.x -= diffX;
 //   clusterLocation.y -= diffY;
//    result.wpts.waypoints.back().x -= diffX;
//    result.wpts.waypoints.back().y -= diffY;
  }

  //this->centerLocation.x -= 1.5;
  //this->centerLocation.y -= 0.5;
  cout << "(SetCenterLocation)Cluster X:  " << clusterLocation.x << ", Cluster Y:  " << clusterLocation.y <<  endl;
  cout << "(SetCenterLocation)Center X:  " << centerLocation.x << ", Center Y:  " << centerLocation.y << endl;
  this->centerLocation.x -= 0.25;
}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}


void SearchController::SetClusterLocation()
{

  clusterAVG.push_back(currentLocation);

  int size = clusterAVG.size();

  float avgX;
  float avgY;
  float avgTheta;

  avgX = clusterAVG[0].x;
  avgY = clusterAVG[0].y;
  avgTheta = clusterAVG[0].theta;

  for(int i = 1; i < size; i++)
  {
    avgX += clusterAVG[i].x;
    //cout << "ClusterAVG[" << i << "].x= " << clusterAVG[i].x << endl;
    avgY += clusterAVG[i].y;
    cout << "ClusterAVG[" << i << "].y= " << clusterAVG[i].y << endl;
    avgTheta += clusterAVG[i].theta;
  }

  clusterLocation.x = avgX/size;
  clusterLocation.y = avgY/size;
  clusterLocation.theta = avgTheta/size;

  clusterLocation.x = clusterLocation.x + (cluster_offset * cos(clusterLocation.theta));
  clusterLocation.y = clusterLocation.y + (cluster_offset * sin(clusterLocation.theta));

  cout << "(SetClusterLocation)Cluster X:  " << clusterLocation.x << ", Cluster Y:  " << clusterLocation.y <<  endl;
}

void SearchController::ResetClusterAVG()
{
  clusterAVG.clear();
}
