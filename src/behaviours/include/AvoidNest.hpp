#ifndef _AVOID_NEST_HPP
#define _AVOID_NEST_HPP

#include "BehaviorManager.hpp"

#include <cmath>

#define NEST_TAG_ID 256

template <typename T>
class AvoidNest : public Behavior
{
private:
   int    _tagsLeft;
   int    _tagsRight;
   bool   _persist;
   bool   _tooClose;
   Action _savedAction;

   T _persistenceTimer;

   void PersistenceCallback()
   {
      _persist = false;
   }

   void TagHandler()
   {
      _tagsRight = 0;
      _tagsLeft  = 0;
      _tooClose = false;
      for(auto tag : _sensors->GetTags())
      {
         if(tag.GetId() != NEST_TAG_ID)
         {
            continue; // skip blocks
         }
         if(tag.Alignment() > 0)
         {
            _tagsRight++;
         }
         else
         {
            _tagsLeft++;
         }

         if(tag.HorizontalDistance() <= 0.2)
         {
            _tooClose = true;
         }
      }
   }
   
public:
   AvoidNest(const SwarmieSensors* sensors) :
      Behavior(sensors),
      _persist(false),
      _persistenceTimer([this]() { this->PersistenceCallback(); }) {}

   ~AvoidNest() {}

   void Update() override
   {
      _action = _llAction;
      TagHandler();
   
      if(_persist)
      {
         _action = _savedAction;
      }

      if(_tooClose)
      {
         _action.drive.left = -100;
         _action.drive.right = -100;
      }

      if(_tagsLeft > 0 || _tagsRight > 0)
      {
         if(_tagsLeft > _tagsRight)
         {
            _action.drive.left = 100;
            _action.drive.right = -100;
         }
         else
         {
            _action.drive.left = -100;
            _action.drive.right = 100;
         }

         _persist = true;
         _savedAction = _action;
         _persistenceTimer.SetInterval(1.2);
         _persistenceTimer.StartOnce();
      }   
   }
};

#endif // _AVOID_NEST_HPP