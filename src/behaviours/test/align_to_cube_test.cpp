#include <gtest/gtest.h>
#include <boost/math/quaternion.hpp>

#include "test_util.h"

#include "AlignToCube.hpp"
#include "AvoidNest.hpp" // for NEST_TAG_ID
#include "SwarmieSensors.hpp"

class AlignToCubeTest : public testing::Test
{
protected:
   SwarmieSensors sensors;
   AlignToCube align;
   boost::math::quaternion<double> defaultOrientation;

   AlignToCubeTest() : align(&sensors), defaultOrientation(1.2, 1.2, 1.2, 2.1) { align.Update(); }
};

TEST_F(AlignToCubeTest, noCubeNoMovement) {
   Action a = align.GetAction();
   EXPECT_FALSE(is_moving(a));
}

TEST_F(AlignToCubeTest, nonCubeTagNoMovement) {
   Tag t = tag_top_left(NEST_TAG_ID);
   sensors.DetectedTag(t);
   align.Update();
   Action a = align.GetAction();
   EXPECT_FALSE(is_moving(a));
}

TEST_F(AlignToCubeTest, cubeOnLeftTurnLeft) {
   sensors.DetectedTag(tag_top_left(CUBE_TAG_ID));
   align.Update();
   for(int i = 0; i < 30 && !is_moving(align.GetAction()); i++) {
      align.Update();
   }
   EXPECT_TRUE(is_turning_left(align.GetAction())) << "left: " << align.GetAction().drive.left
                                                   << " | right: " << align.GetAction().drive.right;

   sensors.ClearDetections();
   align.Update();
   EXPECT_FALSE(is_moving(align.GetAction())) << "left: " << align.GetAction().drive.left
                                              << " | right: " << align.GetAction().drive.right;

   sensors.DetectedTag(tag_bottom_left(CUBE_TAG_ID));
   align.Update();
   for(int i = 0; i < 30 && !is_moving(align.GetAction()); i++) {
      align.Update();
   }
   EXPECT_TRUE(is_turning_left(align.GetAction())) << "left: " << align.GetAction().drive.left
                                                   << " | right: " << align.GetAction().drive.right;
}

TEST_F(AlignToCubeTest, cubeOnRightTurnRight) {
   sensors.DetectedTag(tag_top_right(CUBE_TAG_ID));
   align.Update();
   for(int i = 0; i < 30 && !is_moving(align.GetAction()); i++) {
      align.Update();
   }
   EXPECT_TRUE(is_turning_right(align.GetAction())) << "left: " << align.GetAction().drive.left
                                                    << " | right: " << align.GetAction().drive.right;

   sensors.ClearDetections();
   align.Update();
   EXPECT_FALSE(is_moving(align.GetAction())) << "left: " << align.GetAction().drive.left
                                              << " | right: " << align.GetAction().drive.right;

   sensors.DetectedTag(tag_bottom_right(CUBE_TAG_ID));
   align.Update();
   // accumulate integral term in the PI controller
   for(int i = 0; i < 30 && !is_moving(align.GetAction()); i++) {
      align.Update();
   }
   EXPECT_TRUE(is_turning_right(align.GetAction())) << "left: " << align.GetAction().drive.left
                                                    << " | right: " << align.GetAction().drive.right;
}

TEST_F(AlignToCubeTest, nestAndCubeVisible) {
   sensors.DetectedTag(tag_top_left(CUBE_TAG_ID));
   sensors.DetectedTag(tag_bottom_left(NEST_TAG_ID));
   align.Update();
   EXPECT_FALSE(is_moving(align.GetAction()));
}

// TODO: tag aligned then no turn.