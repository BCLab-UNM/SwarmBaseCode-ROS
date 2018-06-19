#include "AvoidNest.hpp"
#include <cmath>

AvoidNest::AvoidNest(const SwarmieSensors* sensors) :
   Behavior(sensors),
   _persist(false)   
{}

void AvoidNest::TagHandler()
{
   _tagsRight = 0;
   _tagsLeft  = 0;
   _tooClose = false;
   for(auto tag : _sensors->GetTags())
   {
      if(tag.GetId() == 0)
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

void AvoidNest::PersistenceCallback(const ros::TimerEvent& event)
{
   _persist = false;
}

void AvoidNest::Update()
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
      _persistenceTimer = _nh.createTimer(ros::Duration(1.2), &AvoidNest::PersistenceCallback, this, true);
   }   
}