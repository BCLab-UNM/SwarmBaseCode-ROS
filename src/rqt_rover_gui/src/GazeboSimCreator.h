#ifndef GAZEBOSIMCREATOR_H
#define GAZEBOSIMCREATOR_H

#include <QProcess>
#include <QString>
#include <map>
#include <set>
#include <string>

using namespace std;

class GazeboSimCreator
{
public:
    GazeboSimCreator();
    ~GazeboSimCreator();

    QString addGroundPlane( QString ground_name );
    QString removeGroundPlane( QString ground_name );
    QString addRover(QString rover_name, float x, float y, float z);
    QString removeRover(QString rover_name);
    QString startRoverNode(QString rover_name);
    QString startGazebo();
    QString stopGazebo();
    QString removeModel( QString model_name );
    QString addModel(QString model_name, float x, float y, float z);
    QString moveRover(QString rover_name, float x, float y, float z);
    QString applyForceToRover(QString rover_name, float x, float y, float z, float duration);
    bool isLocationOccupied(float x, float y, float clearence);
    bool isGazeboRunning();

private:
    QProcess* gazebo_process;
    QProcess* command_process;
    map<QString, QProcess*> rover_processes;
    set< tuple<float, float> > model_locations;
};

#endif // GAZEBOSIMCREATOR_H
