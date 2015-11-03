#ifndef GAZEBOSIMCREATOR_H
#define GAZEBOSIMCREATOR_H

#include <QProcess>
#include <QString>
#include <map>
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

private:
    QProcess* gazebo_process;
    QProcess* command_process;
    map<QString, QProcess*> rover_processes;
};

#endif // GAZEBOSIMCREATOR_H
