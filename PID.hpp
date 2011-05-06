//
// C++ Interface: PID 
//
// Description: Implementation based on "Control System Design" by Karl Johan Åström 
// implements 'IDEAL TYPE' PID with  
//    -	anti-integrator windup 
//    -	derivative filtering
//    -	setpoint weighing
//    -	bumpless parameter change
//    - 'Backward difference' approximation of parameters
//    - uses the output derivative instead of error derivative
//
//                       1    
// output = K*( e(t) + (----)*e(t) + Td*s*y(t))
//                      Ti*s 
//
//
//GUIDELINES :
//   - for Derivative action
//   		1/Ts >> N/Td 
//   		(Rule of thumb, TsN/Td ~ 0.2 to 0.6)
//
//
//USING THE PID 
//
//
// Author:  <Ajish Babu>, (C) 2011
//
#ifndef CONTROLLERPIDCONTROLLER_H
#define CONTROLLERPIDCONTROLLER_H

#include <math.h>
#include <iostream>
#include <iomanip>
#include <vector>

bool zeroCrossing(double currValue, double prevValue, double refValue = 0);
bool positiveZeroCrossing(double currValue, double prevValue, double refValue = 0);
bool negativeZeroCrossing(double currValue, double prevValue, double refValue = 0);

namespace motor_controller
{
    //structure to hold all controller parameters 
    //for a description of the parameters see below
    struct PIDSettings
    {   
        double Ts,K,Ti,Td,N,B,Tt,YMin,YMax;
        PIDSettings():Ts(0),K(0),Ti(0),Td(0),N(0),B(1),Tt(0),YMin(0),YMax(0){};
    };

    class PID
    {
	public:
	    PID();

	    /* 
	     * Sets the coefficients for a parallel type PID
	     *
	     * A simple PID usage is
	     *
	     *    setParallelCoefficients(Ts, Kp, Ki, Kd) 
	    */
	    void setParallelCoefficients(double _Ts,
		    double _Kp = 0,
		    double _Ki = 0,
		    double _Kd = 0, 
		    double _N = 0,
		    double _B = 1, 
		    double _Tt = -1,
		    double _YMin = 0, 
		    double _YMax = 0);

	    void setIdealCoefficients (double _Ts,
		    double _K = 0 , 
		    double _Ti = 0, 
		    double _Td = 0, 
		    double _N = 0,
		    double _B = 1, 
		    double _Tt = -1,
		    double _YMin = 0, 
		    double _YMax = 0);
            
            void setPIDSettings(const PIDSettings &_settings);

	    double saturate ( double _val );
	    double update ( double _measuredValue, double _referenceValue, double time = 0.0  );
	    void reset();

	    void computeCoefficients();
	    void printCoefficients();


	    // set coefficients before enabling or disabling the compnents below
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
	    	       // < 0 disable
		       // > 0 sets that value
		       // = 0 and Td = 0  disable 
		       // = 0 and Td > 0  Tt = sqrt(Ti * Td) 
		       
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

    // Extracts the step response properties:
    // 		rise time, 
    // 		settling time, 
    // 		percentage overshoot 
    // 		steady state error and
    // 		squared error
    class PIDStepResponseProperties
    {
	public:
	    PIDStepResponseProperties() { reset(); };
	    PIDStepResponseProperties(double _Ts,
		    double _riseTimeFractionReference = 1.0,
		    double _settlingTimeFractionReference = 0.05);

	    void setCoefficients(double _Ts,
		    double _riseTimeFractionReference = 1.0,
		    double _settlingTimeFractionReference = 0.05);

	    void reset();

	    void update(double _actualOutput,
		    double _refInput);

	    void getProperties(double &_riseTimeSec,
		    double &_settlingTimeSec,
		    double &_percentOvershoot,
		    double &_steadyStateError);

	    void printProperties();

	private:
	    double Ts;

	    double riseTimeSec;  // Rise time is seconds
	    double settlingTimeSec; // Settling time in seconds
	    double percentOvershoot; // Percentage Overshoot
	    double steadyStateError; // Steady state error 
	    double steadyStateErrorTimeSec;
	    double squaredError;
	    std::vector<double> maxAmplitudes;
	    double maxAmplitude;

	    double riseTimeFractionReference; // Measures the rise time to the fraction of reference value 
	    double settlingTimeFractionReference; // Measure the settling time limit within the fraction of reference value

	    bool firstRun;
	    double prevOutput;
	    double currTime; // Current time in seconds

	    bool riseTimeDetected;
	    bool firstZeroCrossing;
    };
}

#endif
