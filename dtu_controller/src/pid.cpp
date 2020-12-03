#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double setpoint, double pv );
        void reset_control();

    private:
        double _max;
        double _min;
        double _dt;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
        double _intLimit;
};

PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}

void PID::reset_control()
{
    pimpl->reset_control();
}

PID::~PID() 
{
    delete pimpl;
}

/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0),
    _intLimit(0.7)
{
}

void PIDImpl::reset_control()
{
    _integral = 0;
    _pre_error = 0;
}

double PIDImpl::calculate( double setpoint, double pv )
{
    
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    double kif = 1;
    if( fabs(error) > 0.5 ) kif = 0.4;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral * kif;
    if( Iout > _intLimit ){
        _integral = _intLimit/(_Ki*kif);
        std::cout << "LIMIT+" << std::endl;
    }
    else if( Iout < -_intLimit ) 
    {
        _integral = -_intLimit/(_Ki*kif);
        std::cout << "LIMIT+" << std::endl;
    }
 
    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif
