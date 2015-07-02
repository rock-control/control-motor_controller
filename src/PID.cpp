#include "PID.hpp"

using namespace motor_controller;
using namespace std;

	bool 
zeroCrossing(double currValue, double prevValue, double refValue)
{
    if(positiveZeroCrossing(currValue, prevValue, refValue) ||
	negativeZeroCrossing(currValue, prevValue, refValue) )
	return true;
    return false;
}

	bool 
positiveZeroCrossing(double currValue, double prevValue, double refValue)
{
    if(prevValue < refValue && currValue >= refValue)
	return true;
    return false;
}

	bool 
negativeZeroCrossing(double currValue, double prevValue, double refValue)
{
    if(prevValue > refValue && currValue <= refValue)
	return true;
    return false;
}

void PIDSettings::setParallelCoefficients(double _Kp, double _Ki, double _Kd)
{
    K  = _Kp;
    
    if(_Ki != 0){
        Ti = _Kp / _Ki;
    } else {
        Ti = 0;
    }

    if(_Kp != 0){
        Td = _Kd / _Kp;
    } else {
        Ts = 0;
    }
}

void ParallelPIDSettings::setIdealCoefficients(double _K, double _Ti, double _Td)
{
    Kp = _K;
    if(_Ti != 0){
        Ki = _K / _Ti;
    } else{
        Ki = 0;
    }
    Kd = _Td * _K;
}

PID::PID():
initialized(false),prevValue(0),prevError(0),Bi(0),Ad(0),Bd(0),Ao(0),P(0),
I(0),D(0),rawCommand(0),saturatedCommand(0),bIntegral(false),
bDerivative(false),bDerivativeFiltering(false),Kold(0),Bold(0),
firstRun(true),bSaturated(false),derivativeMode(Error)
{ 
};

void PID::setParallelPIDSettings(const ParallelPIDSettings &_settings){
    setParallelCoefficients(_settings.Ts,
            _settings.Kp,
            _settings.Ki,
            _settings.Kd,
            _settings.N,
            _settings.B,
            _settings.Tt,
            _settings.YMin,
            _settings.YMax);
}

	void 
PID::setParallelCoefficients(double _Ts,
	double _Kp,
	double _Ki,
	double _Kd, 
	double _N,
	double _B, 
	double _Tt,
	double _YMin, 
	double _YMax)
{
    // Cheat, only convert the coefficients using PIDSettings and then call
    // setIdealCoefficients
    PIDSettings settings;
    settings.setParallelCoefficients(_Kp, _Ki, _Kd);
    setIdealCoefficients(_Ts, _Kp, settings.Ti, settings.Td,
	    _N, _B, _Tt, _YMin, _YMax);
}

    void 
PID::setPIDSettings(const PIDSettings &_settings)
{
    if(initialized)
    {
        Kold = gains.K;
        Bold = gains.B;
    }
    else
    {
        Kold = _settings.K;
        Bold = _settings.B;
    }

    gains = _settings;
    autoEnableModes();
    computeCoefficients();
}

	void 
PID::setIdealCoefficients (double _Ts, 
		    double _K,
		    double _Ti, 
		    double _Td, 
		    double _N,
		    double _B, 
		    double _Tt,
		    double _YMin, 
		    double _YMax)
{
    PIDSettings new_settings;
    new_settings.K = _K;
    new_settings.Ti = _Ti;
    new_settings.Td = _Td;
    new_settings.N = _N;
    new_settings.Ts = _Ts;
    new_settings.B = _B;
    new_settings.Tt = _Tt;
    new_settings.YMin = _YMin;
    new_settings.YMax = _YMax;
    setPIDSettings(new_settings);
}

void
PID::autoEnableModes()
{
    if(gains.Ti == 0.0 || isnan(gains.Ti) || isinf(gains.Ti) )   // turns off integral controller
	bIntegral = false;
    else
	bIntegral = true;

    if(gains.Td == 0.0)         // turns off derivative term
	bDerivative = false;
    else
	bDerivative = true;

    if(gains.N == 0.0)          // turns off derviative filtering
	bDerivativeFiltering = false;	
    else
	bDerivativeFiltering = true;
}

	double 
PID::saturate ( double _val )
{
    if ( gains.YMax == 0.0 && gains.YMin == 0.0 ) 
    {
        bSaturated = false;
	return _val;
    }
    else if ( _val > gains.YMax )
    {
        bSaturated = true;
	return gains.YMax;
    }
    else if ( _val < gains.YMin )
    {
        bSaturated = true;
	return gains.YMin;
    }
    else 
    {
        bSaturated = false;
	return _val;
    }
}

	void 
PID::computeCoefficients()
{
    Bi = Ao = Ad = Bd = 0.0;
    if(bIntegral)
    {
	Bi = gains.K*gains.Ts/gains.Ti; // integral gain
	if(gains.Tt == 0 && gains.Td != 0.0) // if Tt is zero and Td nonzero take the ideal value
	    gains.Tt = sqrt(gains.Ti*gains.Td);  // approximate ideal value
	
	if(gains.Tt > 0.0)
	    Ao = gains.Ts/gains.Tt;  // anti-windup
	else 
	    Ao = 0.0;
    }

    if(bDerivative)
    {
	if(bDerivativeFiltering)
	{
	    Ad = gains.Td/(gains.Td+gains.N*gains.Ts);
	    Bd = gains.K*gains.N*Ad; //derivative gain
	}
	else
	{
	    Bd = gains.K*gains.Td/gains.Ts; //derivative gain
	}
    }

    reset();
    initialized = true;
}
    
    void
PID::printCoefficients()
{
    cout << "PID PARAMETERS" << endl;
    cout << " Ideal type parameters " << endl
         << "   Kp = " << gains.K 
         << ",  Ti = " << gains.Ti
         << ",  Td = " << gains.Td << endl; 

    cout << " Parallel type parameters " << endl; 
    cout << "Kp = " << gains.K 
         << ", Ki = " << 1.0 / gains.Ti / gains.K
         << ", Kd = " << gains.Td * gains.K << " >>> " ;

    cout << " Anti-integrator windup gain = " << gains.Tt << endl; 
    cout << " Derivative smoothing factor (N) = " << gains.N << endl; 

    cout << " Internal terms " << endl 
	 << "   Integral Gain = " << Bi 
	 << ",  Anti-windup Gain = " << Ao 
	 << ",  Derivative Gains = " << Ad << "," << Bd << endl << endl;
}

	double 
PID::update ( double _measuredValue, double _referenceValue, double time  )
{
    if(firstRun)
    {
	firstRun = false;
	prevValue = _measuredValue;
	prevReferenceValue = _referenceValue;
	prevError = 0;
    }

    if( (Kold != gains.K || Bold != gains.B) && bIntegral)
    {
	// for bumpless motion on parameter change
	I = I + Kold*(Bold     * _referenceValue - _measuredValue)
	      - gains.K   *(gains.B  * _referenceValue - _measuredValue); 
	Kold = gains.K;
	Bold = gains.B;
    }

    double error = _referenceValue - _measuredValue;

    P = gains.K*(gains.B*_referenceValue - _measuredValue); // Compute proportional part
    I = I + Bi*( error ) // Update integral part
	  + Ao*(saturatedCommand-rawCommand); // Update integral anti-windup

    /**
     * Applies the derivative action either to the error or only to
     * the output
     */
    switch(derivativeMode)
    {
    case Error:
    	D = Ad*D + Bd*( error - prevError );
    	break;
    case Output:
    	D = Ad*D - Bd*(_measuredValue - prevValue);
    	break;
    }

    rawCommand = P + I + D; // compute temporary output
    saturatedCommand = saturate(rawCommand); // Saturation

    prevReferenceValue = _referenceValue;
    prevValue = _measuredValue;
    prevError = error;
    
    return saturatedCommand;
}

	void
PID::reset()
{
    firstRun = true;
    P = I = D = 0.0;
    saturatedCommand = rawCommand = 0.0;
}

	void 
PID::disableIntegral() 
{ 
    bIntegral = false; 
    computeCoefficients(); 
}

	void 
PID::enableIntegral()  
{ 
    bIntegral = true; 
    computeCoefficients();
}

	void 
PID::enableIntegral(double _Ti)  
{ 
    gains.Ti = _Ti; 
    enableIntegral();
} 

	void 
PID::disableDerivative() 
{ 
    bDerivative = false; 
    computeCoefficients();
}

	void 
PID::enableDerivative()  
{ 
    bDerivative = true; 
    computeCoefficients();
}

	void 
PID::enableDerivative(double _Td)  
{ 
    gains.Td = _Td; 
    enableDerivative();
} 

	void 
PID::disableDerivativeFiltering() 
{ 
    bDerivativeFiltering = false; 
    computeCoefficients();
}

	void 
PID::enableDerivativeFiltering()  
{ 
    bDerivativeFiltering = true; 
    computeCoefficients();
} 

	void 
PID::enableDerivativeFiltering(double _N)  
{ 
    gains.N = _N; 
    enableDerivativeFiltering();
} 

        bool
PID::isSaturated()
{
    return bSaturated;
}

PIDState
PID::getState() const
{
    PIDState state;
    state.time = base::Time::now();
    state.P = P;
    state.I = I;
    state.D = D;
    state.input = prevValue;
    state.reference = prevReferenceValue;
    state.rawOutput = rawCommand;
    state.saturatedOutput = saturatedCommand;
    return state;
}


PIDAutoTuning::PIDAutoTuning()
{
    firstRun = true;
    firstZeroCrossing = false;
    currTime = 0.0;
    deltaOutputTime = 0.0;
}

PIDAutoTuning::PIDAutoTuning(double _Ts,
	double _stepRef,
	double _inputAmplitude,
	double _testTimeSec)
{
    firstRun = true;
    firstZeroCrossing = false;
    currTime = 0.0;
    deltaOutputTime = 0.0;
    setCoefficients(_Ts, _stepRef, _inputAmplitude, _testTimeSec);
}

	void 
PIDAutoTuning::setCoefficients(double _Ts,
	double _stepRef,
	double _inputAmplitude,
	double _testTimeSec)
{
    Ts = _Ts;
    inputAmplitude = _inputAmplitude;
    testTimeSec = _testTimeSec; 
    stepRef = _stepRef;
}

	double 
PIDAutoTuning::update(double _measuredValue)
{
    if(firstRun)
    {
	initialValue = _measuredValue;
	maxAmplitude = 0.0;
	minAmplitude = 0.0;
	prevError    = stepRef;
	firstRun = false;
    }

    error = (initialValue + stepRef) - _measuredValue;

    // Extracts the minimum and maximum amplitude
    if(firstZeroCrossing)
    {
	if(error  > maxAmplitude)
	{
		cout << " maxAmp " << maxAmplitude << endl; 
		maxAmplitude = error;
	}
	else if(error < minAmplitude)
	{
		cout << " minAmp " << minAmplitude << endl; 
		minAmplitude = error;
	}
    }

    // Extracts the time between zero crossings
    if( zeroCrossing(error, prevError, 0.0) )
    {
	firstZeroCrossing = true;
	outputTimePeriodSec = 2.0*(deltaOutputTime + Ts);
	cout << " Zero crossing " << currTime 
	     << " delta time " << deltaOutputTime 
	     << " outputTimePeriodSec " << outputTimePeriodSec << endl; 
	deltaOutputTime = 0.0;
    }
    else
	deltaOutputTime += Ts;
    prevError = error;

    // Stops after test time period
    currTime += Ts;
    if(currTime <= testTimeSec) 
    {
	if( error != 0.0 )
	    return ((error) / fabs(error)) * inputAmplitude; 
	else 
	    return inputAmplitude;
    }
    else 
    {
	// computes the control system parameters
	outputAmplitude = (maxAmplitude - minAmplitude) / 2.0;
	ultimateGain = 4.0*fabs(inputAmplitude) / M_PI / outputAmplitude; 
	return 0.0;
    }
}

	void 
PIDAutoTuning::getTunedP(double &_Kp)
{
    _Kp = 0.5 * ultimateGain;
}

	void 
PIDAutoTuning::getTunedPI(double &_Kp, double &_Ti)
{
    _Kp = 0.45 * ultimateGain;
    _Ti = 1.2 / outputTimePeriodSec / _Kp;
}

	void 
PIDAutoTuning::getTunedPID(double &_Kp, double &_Ti, double &_Td)
{
    _Kp = 0.6 * ultimateGain;
    _Ti = 2.0 / outputTimePeriodSec / _Kp;
    _Td = outputTimePeriodSec / 8.0 / _Kp;

    /*
    cout << " Kcu " << ultimateGain 
	 << " Tc " << outputTimePeriodSec 
         << " A " << 	outputAmplitude
         << " Kp " << _Kp 
	 << " Ti " << _Ti 
	 << " Td " << _Td << endl;
	 */
}


PIDStepResponseProperties::PIDStepResponseProperties(double _Ts,
	double _riseTimeFractionReference,
	double _settlingTimeFractionReference)
{
    reset();
    setCoefficients(_Ts, _riseTimeFractionReference, _settlingTimeFractionReference);
}

	void 
PIDStepResponseProperties::setCoefficients(double _Ts,
	double _riseTimeFractionReference,
	double _settlingTimeFractionReference)
{
    Ts = _Ts;
    riseTimeFractionReference = _riseTimeFractionReference;
    settlingTimeFractionReference = _settlingTimeFractionReference;
}

	void
PIDStepResponseProperties::reset()
{
    firstRun = true;
    currTime = 0.0;
    squaredError = 0.0;
    riseTimeDetected = false;
    firstZeroCrossing = false;
    steadyStateErrorTimeSec = 0.0;

    riseTimeSec = 0.0;
    settlingTimeSec = 0.0;
    percentOvershoot = 0.0;
    steadyStateError = 0.0;

    maxAmplitudes.clear();
    maxAmplitude = 0.0;
}

	void 
PIDStepResponseProperties::update(double _actualOutput,
	double _refInput)
{
    if(firstRun)
    {
	firstRun = false;
	prevOutput = _actualOutput;
	maxAmplitude = _actualOutput;
	return;
    }
    currTime += Ts;

    // RISE TIME DETECTION
    if(!riseTimeDetected 
	    && positiveZeroCrossing(_actualOutput, prevOutput, riseTimeFractionReference * _refInput) )
    {
	/*cout << " Rise Time Detected " 
	     << _actualOutput << " " 
	     << prevOutput << endl;
	     */
	riseTimeSec = currTime;
	riseTimeDetected = true;
    }

    // OVERSHOOT DETECTION
    // stores the maximum amplitude after first zero crossing 
    // and before the second zero crossing
    if(positiveZeroCrossing(_actualOutput, prevOutput, _refInput))
    {
	if(!firstZeroCrossing)
	{
	    firstZeroCrossing = true;
	}
	else 
	{
	    maxAmplitudes.push_back(maxAmplitude);
	    maxAmplitude = 0.0;
	}
    }

    if(firstZeroCrossing && (_actualOutput - _refInput) > maxAmplitude)
    {
	maxAmplitude = _actualOutput - _refInput;
	if(maxAmplitudes.empty())
	    percentOvershoot = 100.0 * maxAmplitude /_refInput;
    }

    // SETTLING TIME
    if( fabs(_actualOutput - _refInput) > settlingTimeFractionReference * _refInput )
	settlingTimeSec = currTime;

    // STEADY STATE ERROR
    steadyStateError = (_actualOutput - _refInput);
    if(prevOutput == _actualOutput)
	steadyStateErrorTimeSec += Ts;
    
    // SQUARED ERROR
    squaredError += fabs(_actualOutput - _refInput) * 
	fabs(_actualOutput - _refInput);

    prevOutput = _actualOutput;
}

	void 
PIDStepResponseProperties::getProperties(double &_riseTimeSec,
		    double &_settlingTimeSec,
		    double &_percentOvershoot,
		    double &_steadyStateError)
{
    _riseTimeSec = riseTimeSec;
    _settlingTimeSec = settlingTimeSec;
    _percentOvershoot = percentOvershoot;
    _steadyStateError = steadyStateError; 
}

	void 
PIDStepResponseProperties::printProperties()
{
/*    cout << "\nSTEP RESPONSE PROPERTIES " << endl;
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
    cout << "Total time: " << currTime  << "s" << endl;
    if(!riseTimeDetected)
	cout << " Output did not reach the rise time target !!!" << endl;
    else 
	cout << "Rise time: " << riseTimeSec << "s" << endl;

    if(!firstZeroCrossing)
	cout << " Output did not reach the target !!!" << endl;
    else
    {
	cout << "Percentage overshoot: " << percentOvershoot << "%" << endl;  
	cout << "Amplitudes: " ;
	for(std::vector<double>::iterator i = maxAmplitudes.begin(); i < maxAmplitudes.end(); i++)
	    cout << "[" << *i << "]";
	cout << endl;
    }

    cout << "Settling time: " << settlingTimeSec << "s" << endl; 
    cout << "Steady state error: " << steadyStateError 
	 << " for the last "  << steadyStateErrorTimeSec << " seconds" << endl; 

    cout << "Squared error: " << squaredError << endl;
    cout << "<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    */
    cout
	 << setw(10)
	 << setprecision(5)
	 << "Rt = " << riseTimeSec
	 << ", Ov = " << percentOvershoot  
	 << ", Ts = " << settlingTimeSec 
	 << ", Ess = " << steadyStateError << endl; 
}

