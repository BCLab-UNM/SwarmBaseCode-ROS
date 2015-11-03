#ifndef GAZEBOSIMCREATOR_H
#define GAZEBOSIMCREATOR_H

#include <QProcess>
#include <QString>

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
};

#endif // GAZEBOSIMCREATOR_H
