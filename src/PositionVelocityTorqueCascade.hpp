//
// C++ Interface: PositionVelocityTorqueCascade 
//
// Description: 
//
//
// Author:  <Ajish Babu>, (C) 2011
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef CONTROLLERPVTCONTROLLER_H
#define CONTROLLERPVTCONTROLLER_H

#include <iostream>

#include "PID.hpp"

namespace motor_controller
{
    class PositionVelocityTorqueCascade
    {
	public:
	    PositionVelocityTorqueCascade();
	    PositionVelocityTorqueCascade( 
		    double _Ts,
		    double _Kpp,
		    double _Kpv, double _Kiv,
		    double _Kpt, double _Kit,
		    double _Kalp, 
		    double _Ktv, double _Ktt,
		    double _Kvff = 0, double _Kaff = 0,
		    double _YMin = 0, double _YMax = 0);

	    ~PositionVelocityTorqueCascade();

	    void setSamplingTime (double _Ts); 

	    void setGains ( double _Kpp,
		    double _Kpv, double _Kiv,
		    double _Kpt, double _Kit); // Sets the gains

	    void setGains();

	    void setFeedForwardGain( double _Kvff=0.0, double _Kaff=0.0 );
	    void setVelSmoothingGain( double _Kalp );
	    void setOutputLimits ( double _YMin, double _YMax ); // Sets the max and min output limits
	    void setIntegratorWindupCoeffs (double _Ktv, double _Ktt); // Sets the intergrator wind up coefficients

	    void setPositionController(bool _status) { posController = _status; };
	    void setVelocityController(bool _status) { velController = _status; };

	    double getSmoothenedVelocity() { return velSmooth; };
	    double getVelocity() { return velComputed; };

	    double getVelocityRef() { return velCommand; };
	    double getTorqueRef() { return torCommand; };

            void reset();

	    double update ( double _posMeasured,
		    double _posRef, 
		    double _torMeasured,
		    double _velFF = 0.0, 
		    double _accFF = 0.0 );

            void printCoefficients();


	private:
	    motor_controller::PID PositionP;
	    motor_controller::PID VelocityPI;
	    motor_controller::PID TorquePI;

	    double Ts; // Sampling time

	    double Kpp; // Proportional Position loop
	    double Kpv; // Proportional Velocity loop
	    double Kiv; // Integral Velocity loop
	    double Kpt; // Proportional Torque loop
	    double Kit; // Integral Torque loop

	    double Ktv; // Anti-integrator-windup coefficient velocity
	    double Ktt; // Anti-integrator-windup coefficient torque

	    double Kvff; // Velocity feed forward 
	    double Kaff; // Acceleration feed forward

	    double Kalp;  // Smoothing factor for velocity input 0 <= Ka1 < 1    
	    double YMin, YMax; // Minimum and Macimum output


	    double velPrevStep; // Velocity from previous step for filtering
	    double posPrevStep; // Position from the previous step for velocity calculation
	    double velComputed; // Velocity computed from the position input
	    double velSmooth;  // Smoothened velocity

	    double velCommand; // Generated velocity command
	    double torCommand; // Generated torque command

	    bool posController; // Activate or deactivate position controller
	    bool velController; // Activate or deactivate position and velocity controller

	    bool firstRun;      // To check if it is the first run
    };

}

#endif
