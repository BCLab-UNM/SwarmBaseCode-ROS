#include "GazeboSimCreator.h"
#include <QDir>
#include <string>
#include <unistd.h>
#include <iostream>
#include <utility> // For pair

using namespace std;

GazeboSimCreator::GazeboSimCreator()
{
    gazebo_client_process = NULL;
    gazebo_server_process = NULL;
    command_process = NULL;
}

QString GazeboSimCreator::startGazeboServer()
{
    if (gazebo_server_process != NULL)
        return "Gazebo already running";

    gazebo_server_process = new QProcess();

    QString command = QString("rosrun gazebo_ros gzserver ") + QDir::homePath() + QString("/rover_workspace/simulation/worlds/swarmathon.world");

    cout << "running " + command.toStdString() << endl;

    gazebo_server_process->startDetached(command);//QStringList() << "/home/oDx/Documents/a.txt");

    gazebo_server_process->waitForStarted();
    QString return_msg = ""; //command_process->readAll();
    return return_msg;
}

QString GazeboSimCreator::startGazeboClient()
{
    if (gazebo_client_process != NULL)
        return "Gazebo already running";

    gazebo_client_process = new QProcess();

    QString command = QString("rosrun gazebo_ros gzclient");

    gazebo_server_process->startDetached(command);

    gazebo_client_process->waitForStarted();
    QString return_msg = ""; //command_process->readAll();
    return return_msg;
}


QString GazeboSimCreator::stopGazeboServer()
{
    // Close rover processes
    for(map<QString, QProcess*>::iterator it = rover_processes.begin(); it != rover_processes.end(); it++)
    {
        it->second->close();
        it->second->waitForFinished();
    }

    rover_processes.clear(); // calls destructors for us.

    if (gazebo_server_process == NULL) return "Gazebo is not running";

    QString argument = "~/rover_workspace/cleanup.sh";

    QProcess sh;

    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    cout << return_msg.toStdString() << endl;

    gazebo_server_process->close();
//    gazebo_server_process->waitForFinished();
//    gazebo_server_process->terminate();
//    delete gazebo_server_process;
    delete gazebo_server_process;
    gazebo_server_process = NULL;
      return return_msg;
}

QString GazeboSimCreator::startRoverNode( QString rover_name )
{
    QString argument = "roslaunch ~/rover_workspace/launch/"+rover_name+".launch name:="+rover_name;

    QProcess* rover_process = new QProcess();

    rover_processes.insert(pair<QString, QProcess*>(rover_name, rover_process));

    rover_process->start("sh", QStringList() << "-c" << argument);

    return "rover process spawned";
}

QString GazeboSimCreator::addGroundPlane( QString ground_name )
{
    model_locations.insert(make_tuple(0,0)); // Nest location

    QString argument = "rosrun gazebo_ros spawn_model -sdf -file ~/rover_workspace/simulation/models/" + ground_name + "/model.sdf -model " + ground_name;
    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimCreator::addRover(QString rover_name, float x, float y, float z)
{
    model_locations.insert(make_tuple(x, y));

    QString argument = "rosrun gazebo_ros spawn_model -sdf -file ~/rover_workspace/simulation/models/" + rover_name + "/model.sdf "
            + "-model " + rover_name
            + " -x " + QString::number(x)
            + " -y " + QString::number(y)
            + " -z " + QString::number(z);
            + " -Y " + QString::number(M_PI);

    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimCreator::removeRover( QString rover_name)
{
    QString argument;
    argument = "rosservice call gazebo/delete_model '{model_name: "+rover_name+"}'";

    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimCreator::removeGroundPlane( QString ground_name )
{
    QString argument = "rosservice call gazebo/delete_model '{model_name: "+ground_name+"}'";

    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimCreator::addModel(QString model_name, QString unique_id, float x, float y, float z)
{
    return addModel(model_name, unique_id, x, y, z, 0, 0, 0);
}

QString GazeboSimCreator::addModel(QString model_name, QString unique_id, float x, float y, float z, float roll, float pitch, float yaw)
{
    model_locations.insert(make_tuple(x, y));

    QString argument = "rosrun gazebo_ros spawn_model -sdf -file ~/rover_workspace/simulation/models/" + model_name + "/model.sdf "
            + "-model " + unique_id
            + " -x "    + QString::number(x)
            + " -y "    + QString::number(y)
            + " -z "    + QString::number(z)
            + " -R "    + QString::number(roll)
            + " -P "    + QString::number(pitch)
            + " -Y "    + QString::number(yaw);

    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();


    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimCreator::removeModel( QString model_name )
{
    QString argument = "rosservice call gazebo/delete_model '{model_name: "+model_name+"}'";

    QProcess sh;
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimCreator::moveRover(QString rover_name, float x, float y, float z)
{
            QString x_str = QString::number(x,'f',1);
            QString y_str = QString::number(y,'f',1);
            QString z_str = QString::number(z,'f',1);
           // QString duration_str = QString::number(duration,'f',1);

            QString argument = "rosservice call /gazebo/set_model_state '{model_state: { model_name: alpha, pose: { position: { x: 5, y: 0 ,z: 0 }, orientation: {x: 0, y: 0, z: 1, w: 1 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'";

            QProcess sh;
            sh.start("sh", QStringList() << "-c" << argument);

            sh.waitForFinished();
            QByteArray output = sh.readAll();
            sh.close();

            QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

            return return_msg;
}

QString GazeboSimCreator::applyForceToRover(QString rover_name, float x, float y, float z, float duration)
{
            QString x_str = QString::number(x,'f',1);
            QString y_str = QString::number(y,'f',1);
            QString z_str = QString::number(z,'f',1);
           // QString duration_str = QString::number(duration,'f',1);

            QString argument = "rosservice call /gazebo/apply_body_wrench '{ body_name: \"beta::base_link\", reference_frame: \"beta::base_link\", wrench: { force: { x: 1000, y: 0, z: 0 } }, start_time: 0, duration: 2 }'";

            QProcess sh;
            sh.start("sh", QStringList() << "-c" << argument);

            sh.waitForFinished();
            QByteArray output = sh.readAll();
            sh.close();

            QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

            return return_msg;
}

bool GazeboSimCreator::isLocationOccupied(float x, float y, float clearance)
{
    set<tuple<float, float>>::iterator it;
    for (it = model_locations.begin(); it != model_locations.end(); it++)
    {
        float used_x = get<0>(*it);
        float used_y = get<1>(*it);

        if (fabs(used_x - x) < clearance || fabs(used_y - y) < clearance)
        {
            return true;
        }
    }

    return false;
}

bool GazeboSimCreator::isGazeboServerRunning()
{
    return gazebo_server_process != NULL;
}

GazeboSimCreator::~GazeboSimCreator()
{
    //stopGazeboServer();
    if (gazebo_server_process) gazebo_server_process->close();
    if (gazebo_client_process) gazebo_server_process->close();
    if (command_process) command_process->close();
    delete gazebo_client_process;
    delete gazebo_server_process;
    delete command_process;
}

