/*!
 * \brief   This class is intended as an interface to the Gazebo Simulation. This is acheived
 *          by calling shell commands. A single gazebo process is created that lasts the life of the
 *          program. Other opererations on the simulation are performed by creating a shell process that only
 *          exists as long as the command takes to complete.
 * \author  Matthew Fricke
 * \date    November 11th 2015
 * \todo    A better solution would be to write a gazebo plugin that would pass on gazebo commands.
 *          Using a plugin would reduce the need to manage the shell processes spawned to manipulate models
 *          in the simulation.
 *          addModel can add any model including rovers and ground planes. The addRover and addGroundPlane
 *          functions should just call addModel to avoid duplication of code.
 *          stopGazebo is buggy. It needs to be rewritten so gazebo is closed and the rover nodes shutdown
 *          without closing the GUI nodes.
 * \class   GazeboSimManager
 */

#ifndef GazeboSimManager_H
#define GazeboSimManager_H

#include <QProcess>
#include <QString>
#include <map>
#include <set>
#include <string>
#include <tuple>

using namespace std;

class GazeboSimManager
{
public:
    GazeboSimManager();
    ~GazeboSimManager();

    QString addGroundPlane( QString ground_name );
    QString removeGroundPlane( QString ground_name );
    QString addRover(QString rover_name, float x, float y, float z);
    QString removeRover(QString rover_name);
    QString startRoverNode(QString rover_name);
    QString stopRoverNode(QString rover_name);
    QProcess* startGazeboServer();
    QProcess* startGazeboClient();
    QString stopGazeboServer();
    QString stopGazeboClient();
    QString removeModel( QString model_name );
    QString addModel(QString model_name, QString unique_id, float x, float y, float z, float clearance);
    QString addModel(QString model_name, QString unique_id, float x, float y, float z, float R, float P, float Y, float clearance);
    QString moveRover(QString rover_name, float x, float y, float z);
    QString applyForceToRover(QString rover_name, float x, float y, float z, float duration);
    bool isLocationOccupied(float x, float y, float clearence);
    bool isGazeboServerRunning();
    bool isGazeboClientRunning();
    void cleanUpGazeboClient();
    void cleanUpGazeboServer();

private:
    QString app_root; // Path to the application root directory
    QProcess* gazebo_server_process;
    QProcess* gazebo_client_process;
    QProcess* command_process;
    map<QString, QProcess*> rover_processes;

    // Contains the positions of objects in the simulation and clearance value (the xy plane radius of the object)
    // center x, center y, clearance
    set< tuple<float, float, float> > model_locations;
};

#endif // GazeboSimManager_H
