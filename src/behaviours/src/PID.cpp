#include "PID.hpp"

PID::PID(double k_p) :
   _k_p(k_p),
   _k_i(0),
   _k_d(0),
   _lastError(0),
   _currentError(0),
   _integral(0)
{}

PID::PID(double k_p, double k_i) :
   _k_p(k_p),
   _k_i(k_i),
   _k_d(0),
   _lastError(0),
   _currentError(0),
   _integral(0)
{}

PID::PID(double k_p, double k_i, double k_d) :
   _k_p(k_p),
   _k_i(k_i),
   _k_d(k_d),
   _lastError(0),
   _currentError(0),
   _integral(0)
{}

void PID::Update(double error)
{
   _lastError = _currentError;
   _currentError = error;
   _integral += error;
}

void PID::Reset()
{
   _integral = 0;
   _lastError = 0;
   _currentError = 0;
}

double PID::GetControlOutput() const
{
   double d = _currentError - _lastError;
   return _k_p * _currentError + _k_i * _integral - _k_d * d;
}