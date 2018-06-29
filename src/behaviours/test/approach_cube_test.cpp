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
   EXPECT_FALSE(is_moving(approach.GetAction()));
}

TEST_F(ApproachCubeTest, nestTagAlignedNoMovement)
{
   Tag t(Tag::NEST_TAG_ID, -0.023, 0.01, 0.1, defaultOrientation);
   ASSERT_TRUE(fabs(t.Alignment()) < 0.01) << "Tag not aligned: " << t.Alignment() << std::endl;
   sensors.DetectedTag(t);
   approach.Update();
   EXPECT_FALSE(is_moving(approach.GetAction()));
}

TEST_F(ApproachCubeTest, tagMisalignedNoMovement)
{
   Tag t = tag_top_left(Tag::CUBE_TAG_ID);
   sensors.DetectedTag(t);
   approach.Update();
   EXPECT_FALSE(is_moving(approach.GetAction()));
}