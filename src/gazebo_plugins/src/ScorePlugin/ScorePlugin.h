#ifndef SCORE_PLUGIN_H
#define SCORE_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <thread>

/**
 * This class implements a score counter which keeps track of the number of
 * tags within a square collection zone.
 */
namespace gazebo {

    class ScorePlugin : public ModelPlugin {

        public:

            ~ScorePlugin();

            // required overloaded function from ModelPlugin class
            void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

            // Gazebo actuation function
            void updateWorldEventHandler();
            void collectionZoneContactsEventHandler(ConstContactsPtr& msg);

            // For sending informational messages to the UI
            void sendInfoLogMessage(std::string text);

        private: // functions

            void updateScore();
            std::string loadPublisherTopic();
            void loadUpdatePeriod();
            void loadCollectionZoneSquareSize();

        private: // variables

            physics::Model_V modelList;
            int score;
            float collectionZoneSquareSize;

            // time management variables
            common::Time previousUpdateTime;
            float updatePeriodInSeconds;

            // pointers to gazebo model and xml configuration file
            physics::ModelPtr model;
            sdf::ElementPtr sdf;

            // interface for processing ROS message queue
            event::ConnectionPtr updateConnection;
            std::unique_ptr<ros::NodeHandle> rosNode;

            // ROS Publishers
            ros::Publisher scorePublisher;
            ros::Publisher infoLogPublisher;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ScorePlugin)
}

#endif /* SCORE_PLUGIN_H */
