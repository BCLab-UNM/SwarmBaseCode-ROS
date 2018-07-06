#include <gtest/gtest.h>
#include <boost/math/quaternion.hpp>

#include "test_util.h"

#include "ApproachCube.hpp"
#include "SwarmieSensors.hpp"

class ApproachCubeTest : public testing::Test
{
protected:
   SwarmieSensors sensors;
   ApproachCube approach;
   boost::math::quaternion<double> defaultOrientation;
   ApproachCubeTest() : approach(&sensors), defaultOrientation(1.2, 1.2, 1.2, 2.1) { approach.Update(); }
};

TEST_F(ApproachCubeTest, noCubeNoMovement)
{
   EXPECT_FALSE(is_moving(approach.GetAction()));
}

TEST_F(ApproachCubeTest, nestTagOnlyNoMovement)
{
   Tag t = tag_top_left(Tag::NEST_TAG_ID);
   sensors.DetectedTag(t);
   approach.Update();
   for(int i = 0; i < 30 && !is_moving(approach.GetAction()); i++) {
      approach.Update();
   }
   EXPECT_FALSE(is_moving(approach.GetAction()));
}

TEST_F(ApproachCubeTest, nestTagAlignedNoMovement)
{
   Tag t(Tag::NEST_TAG_ID, -0.023, -0.12, 0.5, defaultOrientation);
   ASSERT_TRUE(fabs(t.Alignment()) < 0.01) << "Tag not aligned: " << t.Alignment() << std::endl;
   sensors.DetectedTag(t);
   approach.Update();
   for(int i = 0; i < 30 && !is_moving(approach.GetAction()); i++) {
      approach.Update();
   }
   EXPECT_FALSE(is_moving(approach.GetAction()));
}

TEST_F(ApproachCubeTest, tagMisalignedNoMovement)
{
   Tag t = tag_top_left(Tag::CUBE_TAG_ID);
   sensors.DetectedTag(t);
   approach.Update();
   for(int i = 0; i < 30 && !is_moving(approach.GetAction()); i++) {
      approach.Update();
   }
   EXPECT_FALSE(is_moving(approach.GetAction()));
}

TEST_F(ApproachCubeTest, tagAlignedMovement)
{
   Tag t(Tag::CUBE_TAG_ID, -0.022, -0.12, 0.4, defaultOrientation);
   ASSERT_TRUE(fabs(t.Alignment()) < 0.01) << "Tag not aligned: " << t.Alignment() << std::endl;
   sensors.DetectedTag(t);
   approach.Update();
   for(int i = 0; i < 30 && !is_moving(approach.GetAction()); i++) {
      approach.Update();
   }
   EXPECT_TRUE(is_moving(approach.GetAction()));
}

TEST_F(ApproachCubeTest, movementStopsWhenTagVanishes)
{
   Tag t(Tag::CUBE_TAG_ID, -0.022, -0.12, 0.4, defaultOrientation);
   sensors.DetectedTag(t);
   approach.Update();
   for(int i = 0; i < 30 && !is_moving(approach.GetAction()); i++) {
      approach.Update();
   }
   EXPECT_TRUE(is_moving(approach.GetAction()));
   sensors.ClearDetections();
   // Update many times to test that the integral term is no longer
   // having an effect.
   for(int i = 0; i < 1000; i++) {
      approach.Update();
   }
   EXPECT_FALSE(is_moving(approach.GetAction()));
}

//TEST_F(ApproachCubeTest, // 2 cubes next to each other one aligned, one to the left slightly, at
// y =~ 2/3 height of frame should be
                         // approaching.

// TEST: Two cubes at same distance, one aligned one close to aliged. Should be moving