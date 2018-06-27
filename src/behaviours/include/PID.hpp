#ifndef _PID_HPP
#define _PID_HPP

class PID
{
private:
   
   double _lastError;
   double _currentError;
   double _integral;

   double _k_p;
   double _k_i;
   double _k_d;

public:
   PID(double k_p);
   PID(double k_p, double k_i);
   PID(double k_p, double k_i, double k_d);

   void   Update(double error);
   double GetControlOutput() const;
   void   Reset();
};

#endif // _PID_HPP