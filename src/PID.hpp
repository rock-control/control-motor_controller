/**Collection of classes related to PID control.
 *
 * Implements classes for 
 * 	- PID
 * 	- PID auto-tuning 
 * 	- extracting step response properties
 *
 * \author  Ajish Babu (ajish.babu@dfki.de)
 */

#ifndef CONTROLLERPIDCONTROLLER_H
#define CONTROLLERPIDCONTROLLER_H

#include <math.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <base/time.h>

//! Detects zero crossing.
bool zeroCrossing(double currValue, double prevValue, double refValue = 0);

//! Detects positive zero crossing.
bool positiveZeroCrossing(double currValue, double prevValue, double refValue = 0);

//! Detects negative zero crossing.
bool negativeZeroCrossing(double currValue, double prevValue, double refValue = 0);

namespace motor_controller
{
    //! Structure to hold the PID parameters for 'IDEAL' type PID. 
    /**
	 * To convert from PARALLEL form to IDEAL form 
	 * 		Ti = 1.0/Ki/Kp and Td = Kd/Kp
	 */
    struct PIDSettings
    {   
		//! Sampling time in seconds
	    double Ts;

		//! Proportional gain 
	    double K; 

		//! Integral time constant, 0 to disable it 
	    double Ti;

		//! Derivative time constant, 0 to disable it 
	    double Td;

		//! Derivative term
		/** Derivative term filtered by a first order system with time constant Td/N 
		 * - Typical values of N are between 8 and 20
		 * - No derivative action on frequencies above N/Td
		 */
	    double N;
		     
		//! Setpoint weighing term
		/** Setpoint weighing term, generally between 0 and 1
		 * - B = 0 reference is introduced only through integral term
		 * - B = 1 disables setpoint weighting
		 */
	    double B;

		//! Anti-integrator-windup
		/** Anti-integrator-windup time constant 
		 * - < 0 disable
		 * - = 0 and Td = 0  disable 
		 * - = 0 and Td > 0  Tt = sqrt(Ti * Td)
		 * */
	    double Tt;		       


		//! Minimum output value 
	    double YMin;
		//! Maximum output value 
	    double YMax;

		//! Constructor
        PIDSettings():Ts(0),K(0),Ti(0),Td(0),N(0),B(1),Tt(-1),YMin(0),YMax(0){};

        bool operator==(const PIDSettings& other) const{
            return 
                Ts == other.Ts &&
                K == other.K &&
                Ti == other.Ti &&
                Td == other.Td &&
                N == other.N &&
                B == other.B &&
                Tt == other.Tt &&
                YMin == other.YMin &&
                YMax == other.YMax;
        }

        bool operator!=(const PIDSettings& other) const{
            return !(*this == other);
        }

	    void setParallelCoefficients(
		    double _Kp = 0,
		    double _Ki = 0,
		    double _Kd = 0);
    };

    /** Representation of the internal state of a PID controller
     *
     * This is meant to be used for debugging / logging purposes mainly
     */
    struct PIDState
    {
        base::Time time;

        bool initialized;
        double P, I, D;
        double input;
        double rawOutput;
        double saturatedOutput;
    };

	/**
	 * \brief PID Implementation in C++
	 *
	 * \details 
	 * Implementation based on "Control System Design" by Karl Johan Åström. 
	 * implements 'IDEAL TYPE' PID with  
	 *    -	anti-integrator windup 
	 *    -	derivative filtering
	 *    -	setpoint weighing
	 *    -	bumpless parameter change
	 *    - 'Backward difference' approximation of parameters
	 *    - uses the output derivative instead of error derivative
	 *
	 * Ideal design: \f$u(t) = K_p \left\{ e(t) + \frac{1}{T_i}\int e(t) + T_d \dot{y}(t)) \right\} \f$
	 * where \f$u(t)\f$ is the controller output, \f$e(t)\f$ is the error and \f$y(t)\f$ is measured control variable.
	 *
	 * USAGE:
	 *  - A simple PID usage is 
	 * 	  <code>
     *    setParallelCoefficients(Ts, Kp, Ki, Kd) 
	 * 	  </code>
	 * 	  which uses the parallel form of PID(as seen in most textbooks). In this case almost all the other features of the controller are disabled.
	 *
	 * GUIDELINES :
	 *   - Derivative action
	 *   		\f$\frac{1}{T_s} >> \frac{N}{T_d}\f$
	 *   		( Rule of thumb, \f$ 0.2 \leq \frac{T_sN}{T_d} \leq 0.6} \f$ )
	 *
	 * \author  Ajish Babu (ajish.babu@dfki.de)
	 */
    class PID
    {
	public:
		//! Constructor
	    PID();

	    //! Sets the coefficients for a 'PARALLEL' type PID
		/**
		 * Parallel design: \f$u(t) = K_p e(t) + K_i\int e(t) + K_d \dot{y}(t))\f$
		 * where \f$u(t)\f$ is the controller output, \f$e(t)\f$ is the error and \f$y(t)\f$ is measured control variable.
		 *
		 * \param _Kp proportional gain
		 * \param _Ki integral gain
		 * \param _Kd derivative gain
		 * 
		 * see \c PIDSettings for details of the rest of the parameters
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

	    //! Sets the coefficients for a 'PARALLEL' type PID
	    /**
		 * Ideal design: \f$u(t) = K_p \left\{ e(t) + \frac{1}{T_i}\int e(t) + T_d \dot{y}(t)) \right\} \f$
		 * where \f$u(t)\f$ is the controller output, \f$e(t)\f$ is the error and \f$y(t)\f$ is measured control variable.
		 *
		 * \param _Kp gain for proportional error, integral error and derivative output
		 * \param _Ti integral time constant
		 * \param _Td derivative time constant
		 *
		 * see \c PIDSettings for details of the rest of the parameters
	     */
	    void setIdealCoefficients (double _Ts,
		    double _K = 0 , 
		    double _Ti = 0, 
		    double _Td = 0, 
		    double _N = 0,
		    double _B = 1, 
		    double _Tt = -1,
		    double _YMin = 0, 
		    double _YMax = 0);
            
	    //! Sets the coefficients for a 'IDEAL' type PID using the \c struct \c PIDSettings 
        void setPIDSettings(const PIDSettings &_settings);

	    //! Sets the saturation limit to +/- \c _val, disables if \c _val = 0.0
	    double saturate ( double _val );
	    //! Returns the Controller command at each sampling time
	    /**
		 * \param _measuredValue is the measured system output
		 * \param _referenceValue is the reference input for the controller
		 * \param time is the optional current time
	     */
	    double update ( double _measuredValue, double _referenceValue, double time = 0.0  );

	    //! Resets the controller
	    void reset();

	    //! Prints the controller coefficients
	    void printCoefficients();


	    //! Diables the integral part of the controller 
		void disableIntegral(); 

	    //! Enables the integral part of the controller 
	    void enableIntegral(); 

	    //! Enables the integral part of the controller with time constant \c _Ti
	    void enableIntegral(double _Ti); 


	    //! Diables the derivative part of the controller 
	    void disableDerivative(); 

	    //! Enables the derivative part of the controller 
	    void enableDerivative(); 

	    //! Enables the derivative part of the controller with time constant \c _Td 
	    void enableDerivative(double _Td); 


	    //! Diables the derivative filtering 
	    void disableDerivativeFiltering(); 

	    //! Enables the derivative filtering 
	    void enableDerivativeFiltering(); 

	    //! Enables the derivative filtering with time constant \c _N
	    void enableDerivativeFiltering(double _N)  ; 

        //! Returns true if the controller got saturated in the last run
        bool isSaturated();

        //! Returns the internal state of the controller
        PIDState getState() const;

	private:

		//! PID gains 
		struct PIDSettings gains;

		//! true if coefficients initialized atleast once
	    bool initialized;

		//! Error from previous step  
	    double prevValue;

		//! Internal coefficients  
	    double Bi, Ad, Bd, Ao; 
		//! Internal variables  
	    double P, I, D, rawCommand, saturatedCommand;

		//! false turns off Integral term
	    bool bIntegral;
		//! false turns off Derivative term
	    bool bDerivative;
		//! false turns off Derivative Filtering
	    bool bDerivativeFiltering;

		//! Bumpless parameter change- old value of K if parameter changed
	    double Kold;
		//! Bumpless parameter change- old value of B if parameter changed
	    double Bold;

		//! true if first run 
	    bool firstRun;

            //! true if the controller is saturated
            bool bSaturated;

	    //! Computes the controller coefficients
	    void computeCoefficients();

        //! Enables or disables the various modes (integral, ...) based on
        // whether the corresponding gains are set
        void autoEnableModes();
    };

    //! PID auto tuning using Relay Feedback 
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

    /** Extracts the step response properties
	 * 
	 * Properties:
     * 	- rise time, 
     * 	- settling time, 
     * 	- percentage overshoot 
     * 	- steady state error
     * 	- squared error
	 */
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
