#ifndef SCORE_PLUGIN_H
#define SCORE_PLUGIN_H

#include <gazebo/gazebo.hh>

/**
 * This class implements a score counter which keeps track of the number of
 * tags within the nest radius.
 */
namespace gazebo {
    class ScorePlugin : public ModelPlugin {
        public:
            // required overloaded function from ModelPlugin class
            void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ScorePlugin)
}

#endif /* SCORE_PLUGIN_H */
