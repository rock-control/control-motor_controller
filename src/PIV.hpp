//
// C++ Interface: PIV 
//
// Description: 
//
//
// Author:  <Ajish Babu>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef CONTROLLERPIVCONTROLLER_H
#define CONTROLLERPIVCONTROLLER_H

#include "SimpleIntegrator.hpp"

namespace motor_controller
{
    struct PIVSettings {
        double Kpp;  //!< Position Proportional Gain.
        double Kiv;  //!< Velocity Integral Gain.
        double Kpv;  //!< Velocity Proportional Gain.
        double Kvff; //!< Velocity Feedforward Gain.
        double Kaff; //!< Acceleration Feedforward Gain.
        double Kalp; //!< Velocity Estimation Smoothing Factor.
        double Ts;   //!< Sampling Time.
        double YMin; //!< Lower Output Limit.
        double YMax; //!< Upper Output Limit.
        double Kt;   //!< Anti-Windup Coefficient.
        PIVSettings() : Kpp(0),Kiv(0),Kpv(0),Kvff(0),Kaff(0),Kalp(0),Ts(0.1),
            YMin(0), YMax(0), Kt(0) {}      
    };

    class PIV
    {
	public:
	    PIV();
	    PIV( double _Kpp, double _Kiv, double _Kpv, 
		    double _Kvff, double _Kaff,
		    double _Kalp, 
		    double _Ts, 
		    double _YMin = 0, double _YMax = 0, 
		    double _Kt = 0);
        PIV ( const struct PIVSettings& _settings );

	    ~PIV();

	    void setGains ( double _Kpp, double _Kiv, double _Kpv ); // Sets the PIV gains
	    void setFeedForwardGain( double _Kvff=0.0, double _Kaff=0.0 ){ Kvff = _Kvff; Kaff = _Kaff; };
	    void setVelSmoothingGain( double _Kalp )  { Kalp = _Kalp; };
	    void setSamplingTime (double _Ts); 
	    void setOutputLimits ( double _YMin, double _YMax ){ YMin = _YMin; YMax = _YMax; }; // Sets the max and min output limits
	    void setPositionController(bool _status) { posController = _status; };
	    void setIntegratorWindupCoeff (double _Kt) { Kt = _Kt; }; // Sets the intergrator wind up coefficients
      void setPIVSettings ( const struct PIVSettings& _settings );
	    double getSmoothenedVelocity() { return velSmooth; };
	    double getVelocity() { return velComputed; };
            bool isSaturated() { return bSaturated; }

	    double saturate_windup(double _value); // Saturates the input based on YMin and YMax returns the excess value
	    double saturate ( double _val );
            void reset();

	    double updateVelLoop ( double _velMeasured, double _velCmd, double _posCmd, double accFF=0.0 ); // update velocity loop
	    double updatePosLoop ( double _posError ) const { return Kpp * (_posError); }; // updates position loop

	    double update ( double _posMeasured, double _posRef, double _velFF = 0.0, double _accFF = 0.0 );

            void printCoefficients();


	private:
	    double Kpp; // Proportional Position loop
	    double Kiv; // Intergral Velocity loop
	    double Kpv; // Proportional Position loop

	    double Kvff; // Velocity feed forward 
	    double Kaff; // Acceleration feed forward
	    double Kalp;  // Smoothing factor for velocity input 0 <= Ka1 < 1    

	    double Kt; // Anti-integrator-windup coefficient
	    double limitDiff; // Stores the difference in the saturation

	    double Ts; // Sampling time

	    double YMax; // Maximum output value
	    double YMin; // Minimum output value

	    double velPrevStep; // Velocity from previous step for filtering
	    double posPrevStep; // Position from the previous step for velocity calculation

	    bool posController; // Activate or deactivate position controller
	    bool firstRun;      // To check if it is the first run
            bool bSaturated;    // True if the last run was saturated

	    double posCommand; // Generated position command
	    double velComputed; // Velocity computed from the position input

	    double velSmooth;  // Smoothened velocity
	    double velCommand; // the velocity command

	    SimpleIntegrator velITerm; // Integral term
    };

}

#endif
