//
// C++ Interface: PID 
//
// Description: Implementation based on "Control System Design" by Karl Johan Åström 
// implements 'interacting type' PID with  
//    -	anti-integrator windup 
//    -	derivative filtering
//    -	setpoint weighing
//    -	bumpless parameter change
//    - 'Backward difference' approximation of parameters
//
//                       1    
// output = K*( e(t) + (----)*e(t) + Td*s*y(t))
//                      Ti*s 
//
//
//GUIDELINES :
//   - for Derivative action
//   		1/Ts >> N/Td 
//   		(Rule of thumb, hN/Td ~ 0.2 to 0.6)
//    
//
//
//
// Author:  <Ajish Babu>, (C) 2011
//
#ifndef CONTROLLERPIDCONTROLLER_H
#define CONTROLLERPIDCONTROLLER_H

#include <math.h>
#include <iostream>

namespace motor_controller
{
    class PID
    {
	public:
	    PID();
	    PID(double _Ts,
		    double _K = 0, 
		    double _Ti = 0, 
		    double _Td = 0, 
		    double _N = 0,
		    double _B = 1, 
		    double _Tt = 0,
		    double _YMin = 0, 
		    double _YMax = 0);

	    void setCoefficients (double _Ts,
		    double _K = 0 , 
		    double _Ti = 0, 
		    double _Td = 0, 
		    double _N = 0,
		    double _B = 1, 
		    double _Tt = 0,
		    double _YMin = 0, 
		    double _YMax = 0);

	    double saturate ( double _val );
	    double update ( double _measuredValue, double _referenceValue  );

	    void computeCoefficients();

	    void disableIntegral(); 
	    void enableIntegral(); 
	    void enableIntegral(double _Ti); 

	    void disableDerivative(); 
	    void enableDerivative(); 
	    void enableDerivative(double _Td); 

	    void disableDerivativeFiltering(); 
	    void enableDerivativeFiltering(); 
	    void enableDerivativeFiltering(double _N)  ; 

	private:

	    bool initialized; //true if coefficients initialized atleast once

	    double K; // Proportional 
	    double Ti; // Integral time constant .. 0 to disable it  
	    double Td; // Derivative time constant .. 0 to disable it
	    double N; // Derivative term filtered by a first order system with time constant Td/N
	 	      // Typical values of N are 8 to 20
		      // No derivative action on frequencies above N/Td
		     
	    double B; // setpoint weighing term, generally between 0 and 1
	   	      // b = 0 reference is introduced only through integral term
		      // b = 1 in-effect disable setpoint weighting

	    double Tt; // Anti-integrator-windup time constant 
	    double Ts; // Sampling time

	    double YMax; // Maximum output value
	    double YMin; // Minimum output value

	    double prevValue; // Error from previous step 

	    double Bi, Ad, Bd, Ao; // internal coefficients
	    double P, I, D, rawCommand, saturatedCommand; // internal variables

	    bool bIntegral; // false turns off Integral term
	    bool bDerivative; // false turns off Derivative term
	    bool bDerivativeFiltering; // false turns off Derivative Filtering 

	    // for bumpless parameter change
	    double Kold;  // old value of K if parameter changed
	    double Bold;  // old value of B if parameter changed

	    bool firstRun; // true if first run

    };

    // PID auto tuning using Relay Feedback
    class PIDAutoTuning
    {
	public:
	    PIDAutoTuning();
	    PIDAutoTuning(double _Ts,
		    double _stepRef = 1.0,
		    double _inputAmplitude = 0.1,
		    double _testTimeSec = 10.0);

	    void setCoefficients(double _Ts,
		    double _stepRef = 1.0,
		    double _inputAmplitude = 0.1,
		    double _testTimeSec = 10.0);

	    double update(double _measuredValue);

	    void getTunedP(double &_Kp); // Tuned value for P controller
	    void getTunedPI(double &_Kp, double &_Ti); // Tuned values for interacting PI controller
	    void getTunedPID(double &_Kp, double &_Ti, double &_Td); // Tuned values for interacting PID controller

	private:
	    double Ts; // Sampling time 
	    double testTimeSec; // Total time period of the test

	    double stepRef; // Step reference input
	    double inputAmplitude; // Amplitude of input signal

	    double outputTimePeriodSec; // Time period of output signal
	    double outputAmplitude; // Amplitude of output signal

	    double initialValue, // Start value of the output
		   prevError; // Measured value in the last step

	    double error;

	    bool firstRun; // true if first run
	    bool firstZeroCrossing;

	    double currTime; // Current time
	    double deltaOutputTime; // Delta time for output wave

	    double maxAmplitude, minAmplitude; // Minimum and maximum amplitude

	    double ultimateGain;  // Gain for computation of parameters
    };

}

#endif
