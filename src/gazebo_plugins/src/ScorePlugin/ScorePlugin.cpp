#include "ScorePlugin.h"

using namespace gazebo;
using namespace std;

// required overloaded function from ModelPlugin class
void ScorePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    score = 0;
    model = _model;
    sdf = _sdf;

    // set the update period (number of updates per second) for this plugin
    previousUpdateTime = model->GetWorld()->GetSimTime();
    loadUpdatePeriod();
    loadCollectionZoneRadius();

    // Create a ros node
    rosNode.reset(new ros::NodeHandle(string(model->GetName()) + "_score"));

    // Create publishers so we can send info messages to the UI
    scorePublisher = rosNode->advertise<std_msgs::String>(loadPublisherTopic(), 1, true);
    infoLogPublisher = rosNode->advertise<std_msgs::String>("/infoLog", 1, true);

    // Connect the updateWorldEventHandler function to Gazebo;
    // ConnectWorldUpdateBegin sets our handler to be called at the beginning of
    // each physics update iteration
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ScorePlugin::updateWorldEventHandler, this)
    );
}

// Gazebo actuation function
void ScorePlugin::updateWorldEventHandler() {
    common::Time currentTime = model->GetWorld()->GetSimTime();

    if((currentTime - previousUpdateTime).Float() < updatePeriodInSeconds) {
        return;
    }
    previousUpdateTime = currentTime;

    updateScore();
    std_msgs::String msg;
    msg.data = std::to_string(score);
    scorePublisher.publish(msg);
}

/**
 * updates the score based on the proximity of tag models in the tagModels list.
 */
void ScorePlugin::updateScore() {
    math::Pose nestPose = model->GetWorldPose();
    physics::Model_V m = model->GetWorld()->GetModels();

    score = 0;

    for(unsigned int i = 0; i < m.size(); i++) {
        if (m[i]->GetName().substr(0,2).compare("at") == 0) {
            if(m[i]->GetWorldPose().pos.Distance(nestPose.pos) <= collectionZoneRadius) {
                score++;
            }
        }
    }
}

/**
 * This function is used to send an info log message to the RQT GUI.
 */
void ScorePlugin::sendInfoLogMessage(string text) {
 std_msgs::String msg;
 msg.data = model->GetName() + ": " + text;
 infoLogPublisher.publish(msg);
}

/**
 * This function loads the publisher topic to be used by this plugin from the
 * configuration XML file.
 */
std::string ScorePlugin::loadPublisherTopic() {
    string topic;

    if(sdf->HasElement("scoreTopic")) {
        topic = sdf->GetElement("scoreTopic")->Get<std::string>();
    } else {
        ROS_ERROR_STREAM("[Score Plugin : " << model->GetName()
            << "]: In ScorePlugin.cpp: loadPublisherTopic(): No <scoreTopic> "
            << "tag is defined in the model SDF file");
        exit(1);
    }

    return topic;
}

/**
 * This function loads the update rate from the SDF configuration file and uses
 * that value to set the update period. Effectively, the updatePeriod variable
 * defines how many times per second the plugin will publish.
 */
void ScorePlugin::loadUpdatePeriod() {
  float updateRate = 1.0;

  if(!sdf->HasElement("updateRate")) {
    ROS_INFO_STREAM("[Score Plugin : " << model->GetName()
      << "]: In ScorePlugin.cpp: loadUpdatePeriod(): "
      << "missing <updateRate> tag, defaulting to 0.2");
    updateRate = 0.2;
  } else {
    updateRate = sdf->GetElement("updateRate")->Get<float>();

    // fatal error: the update cannot be <= 0 and especially cannot = 0
    if(updateRate <= 0) {
      ROS_ERROR_STREAM("[Score Plugin : " << model->GetName()
        << "]: In ScorePlugin.cpp: loadUpdatePeriod(): "
        << "updateRate = " << updateRate << ", updateRate cannot be <= 0.0");
      exit(1);
    }
  }

  // set the update period for this plugin: the plugin will refresh at a rate
  // of "updateRate" times per second, i.e., at "updatePeriodInSeconds" hertz
  updatePeriodInSeconds = 1.0 / updateRate;
}

/**
 * This function loads the collection zone radius from the SDF configuration
 * file. Effectively, the collectionZoneRadius variable defines how many far
 * away a tag must be from the nest to count towards the score value.
 */
void ScorePlugin::loadCollectionZoneRadius() {
  if(!sdf->HasElement("collectionZoneRadius")) {
    ROS_INFO_STREAM("[Score Plugin : " << model->GetName()
      << "]: In ScorePlugin.cpp: loadCollectionZoneRadius(): "
      << "missing <collectionZoneRadius> tag, defaulting to 0.525");
    collectionZoneRadius = 0.525;
  } else {
    collectionZoneRadius = sdf->GetElement("collectionZoneRadius")->Get<float>();

    // fatal error: the radius cannot be <= 0 and especially cannot = 0
    if(collectionZoneRadius <= 0) {
      ROS_ERROR_STREAM("[Score Plugin : " << model->GetName()
        << "]: In ScorePlugin.cpp: loadCollectionZoneRadius(): "
        << "collectionZoneRadius = " << collectionZoneRadius
        << ", collectionZoneRadius cannot be <= 0.0");
      exit(1);
    }
  }
}

ScorePlugin::~ScorePlugin() {
    rosNode->shutdown(); // Shutdown the ROS node

    // Stop the multi-threaded ROS spinner
    gazebo::shutdown();
}
