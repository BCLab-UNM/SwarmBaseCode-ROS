#include "GazeboSimCreator.h"
#include <string>
#include <unistd.h>
#include <iostream>

using namespace std;

GazeboSimCreator::GazeboSimCreator()
{
    gazebo_process = NULL;
    command_process = NULL;
}

QString GazeboSimCreator::startGazebo()
{
    if (gazebo_process != NULL)
        return "Gazebo already running";

    gazebo_process = new QProcess();
    gazebo_process->startDetached("rosrun gazebo_ros gazebo");//QStringList() << "/home/oDx/Documents/a.txt");

    QString return_msg = ""; //command_process->readAll();
    return return_msg;
}

QString GazeboSimCreator::stopGazebo()
{
    if (gazebo_process == NULL) return "Gazebo is not running";

    QString argument = "~/rover_workspace/src/rover_driver_gazebo_launch/cleanup.sh";

    QProcess sh;

    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    cout << return_msg.toStdString() << endl;

    gazebo_process->close();
//    gazebo_process->waitForFinished();
//    gazebo_process->terminate();
//    delete gazebo_process;
    delete gazebo_process;
    gazebo_process = NULL;
      return return_msg;
}

QString GazeboSimCreator::startRoverNode( QString rover_name )
{
    QString argument = "roslaunch ~/rover_workspace/src/rover_driver_gazebo_launch/launch_files/"+rover_name+".launch name:="+rover_name;

    QProcess* rover_process = new QProcess();

    rover_process->start("sh", QStringList() << "-c" << argument);

    return "rover process spawned";
}

QString GazeboSimCreator::addGroundPlane( QString ground_name )
{
    QString argument = "./rover_driver_node_launch " + ground_name;

    QProcess sh;
    sh.setWorkingDirectory("~/rover_workspace/src/rover_driver_gazebo_launch");
    sh.start("sh", QStringList() << "-c" << argument);

    sh.waitForFinished();
    QByteArray output = sh.readAll();
    sh.close();

    QString return_msg = "<br><font color='yellow'>" + output + "</font><br>";

    return return_msg;
}

QString GazeboSimCreator::addRover(QString rover_name, float x, float y, float z)
{
    QString argument = "rosrun gazebo_ros spawn_model -sdf -file ~/rover_workspace/src/rover_misc/gazebo/models/" + rover_name + "/model.sdf -model " + rover_name + " -x " + QString::number(x) + " -y" + QString::number(y)+ " -z" + QString::number(z);
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

GazeboSimCreator::~GazeboSimCreator()
{
    stopGazebo();
    if (gazebo_process) gazebo_process->close();
    if (command_process) command_process->close();
    delete gazebo_process;
    delete command_process;
}

