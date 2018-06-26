#include <gtest/gtest.h>
#include <boost/math/quaternion.hpp>

#include "test_util.h"

#include "AvoidNest.hpp"
#include "SwarmieSensors.hpp"
#include "ROSTimer.hpp"

class AvoidNestTest : public testing::Test
{
protected:
   SwarmieSensors sensors;
   AvoidNest<ROSTimer> avoid;

   AvoidNestTest() : avoid(&sensors) {
      avoid.Update();
   }
};

TEST_F(AvoidNestTest, noTagsNoAction) {
   Action a = avoid.GetAction();
   EXPECT_FALSE(is_moving(a));
}

TEST_F(AvoidNestTest, nonCenterTagNoAction) {
   Tag t(0, 1, 1, 1, boost::math::quaternion<double>(1.2, 1.2, 1.2, 2.1));
   sensors.DetectedTag(t);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_FALSE(is_moving(a));
}

TEST_F(AvoidNestTest, centerTagTriggersMovement) {
   Tag t(NEST_TAG_ID, 1, 1, 1, boost::math::quaternion<double>(1.2, 1.2, 1.2, 2.1));
   sensors.DetectedTag(t);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_TRUE(is_moving(a));
}
