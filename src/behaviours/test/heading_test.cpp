#include <gtest/gtest.h>
#include "Heading.hpp"

#include <cmath>

using namespace core;

TEST(HeadingTest, zeroEquivalentHeadings)
{
   Heading h0(0);
   Heading h1(2*M_PI);
   Heading h2(-2*M_PI);
   Heading h3(300*M_PI);

   EXPECT_TRUE(h0 == h1);
   EXPECT_TRUE(h0 == h2);
   EXPECT_TRUE(h0 == h3);
}

TEST(HeadingTest, additionCommutes)
{
   Heading h1(2.7);
   Heading h2(1.3);

   EXPECT_TRUE(h1+h2 == h2+h1);
}

TEST(HeadingTest, transitiveEquality)
{
   Heading h1(M_PI);
   Heading h2(-M_PI);
   Heading h3(3*M_PI);

   EXPECT_TRUE(h1 == h2);
   EXPECT_TRUE(h2 == h3);
   EXPECT_TRUE(h1 == h3);
}

TEST(HeadingTest, additiveInverse)
{
   Heading h1(2.7);
   Heading h2 = Heading(0) - h1;
   EXPECT_TRUE(Heading(0) == h1 + h2);
}
