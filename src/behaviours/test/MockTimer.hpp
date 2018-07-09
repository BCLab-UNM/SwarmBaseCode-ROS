#ifndef _MOCK_TIMER_HPP
#define _MOCK_TIMER_HPP

#include "gmock/gmock.h"
#include "Timer.hpp"

class MockTimer : public Timer
{
public:
   MOCK_METHOD0(Stop, void());
   MOCK_METHOD0(StartOnce, void());
   MOCK_CONST_METHOD0(Expired, bool());
   MOCK_METHOD1(SetInterval, void(int x));
};

#endif // _MOCK_TIMER_HPP