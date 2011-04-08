//
// C++ Implementation: PID
//
// Description:
//
//
// Author:  <Ajish Babu>, (C) 2011
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "PID.hpp"

using namespace motor_controller;
using namespace std;

PID::PID()
{ 
    bIntegral = true;
    bDerivative = true;
    bDerivativeFiltering = true;	

    initialized = false; 
    firstRun = true;
};

PID::PID (double _Ts, 
		    double _K,
		    double _Ti, 
		    double _Td, 
		    double _N,
		    double _B, 
		    double _Tt,
		    double _YMin, 
		    double _YMax)
{
    bIntegral = true;
    bDerivative = true;
    bDerivativeFiltering = true;	

    setCoefficients(_K, _Ti, _Td, _N, _Ts, _B, _Tt, _YMin, _YMax);
    firstRun = true;
}

	void 
PID::setCoefficients (double _Ts, 
		    double _K,
		    double _Ti, 
		    double _Td, 
		    double _N,
		    double _B, 
		    double _Tt,
		    double _YMin, 
		    double _YMax)
{
    if(initialized)
    {
	Kold = K;
	Bold = B;
    }
    else
    {
	Kold = _K;
	Bold = _B;
    }

    K = _K;
    Ti = _Ti;
    Td = _Td;
    N = _N;
    Ts = _Ts;
    B = _B;
    Tt = _Tt;
    YMin = _YMin;
    YMax = _YMax;

    computeCoefficients();
}

	double 
PID::saturate ( double _val )
{
    if ( YMax == 0.0 && YMin == 0.0 ) 
	return _val;
    else if ( _val > YMax )
	return YMax;
    else if ( _val < YMin )
	return YMin;
    else 
	return _val;
}

	double 
PID::update ( double _measuredValue, double _referenceValue  )
{
    if(firstRun)
    {
	firstRun = false;
	prevValue = _measuredValue;
    }

    if(Kold != K || Bold != B)
    {
	// for bumpless motion on parameter change
	I = I + Kold*(Bold * _referenceValue - _measuredValue)
	      - K    *(B     * _referenceValue - _measuredValue); 
	Kold = K;
	Bold = B;
    }

    P = K*(B*_referenceValue - _measuredValue); // Compute proportional part
    I = I + Bi*(_referenceValue - _measuredValue) // Update integral part
	  + Ao*(saturatedCommand-rawCommand); // Update integral anti-windup
    D = Ad*D - Bd*(_measuredValue - prevValue); // Compute derivative part

    rawCommand = P + I + D; // compute temporary output
    saturatedCommand = saturate(rawCommand); // Saturation

    prevValue = _measuredValue; //update old process output

    cout 
	<< "  y " << _measuredValue
	<< ", ysp " << _referenceValue
	<< ", err " << _referenceValue - _measuredValue
	<< ", P " << P 
	<< ", I " << I 
	<< ", D " << D 
	<< ", rC " << rawCommand 
	<< ", sC " << saturatedCommand << endl;

    return saturatedCommand;
}

	void 
PID::computeCoefficients()
{
    if(Tt == 0.0)
	Tt = sqrt(Ti*Td);  // approximate ideal value

    if(Ti == 0.0)	   // turns off integral controller
	bIntegral = false;

    if(Td == 0.0)         // turns off derivative term
	bDerivative = false;

    if(N == 0.0)          // turns off derviative filtering
	bDerivativeFiltering = false;	

    Bi = Ao = Ad = Bd = 0.0;
    if(bIntegral)
    {
	Bi = K*Ts/Ti; // integral gain
	Ao = Ts/Tt;  // anti-windup
    }

    if(bDerivative)
    {
	if(bDerivativeFiltering)
	{
	    Ad = Td/(Td+N*Ts);
	    Bd = K*N*Ad; //derivative gain
	}
	else
	{
	    Bd = K*Td/Ts; //derivative gain
	}
    }

//    cout << " Bi " << Bi << " Ao " << Ao << " Ad " << Ad << " Bd " << Bd << endl;
    P = I = D = 0.0;
    initialized = true;
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
    Ti = _Ti; 
    bIntegral = true; 
    computeCoefficients();
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
    Td = _Td; 
    bDerivative = true; 
    computeCoefficients();
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
    N = _N; 
    bDerivativeFiltering = true; 
    computeCoefficients();
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
	    maxAmplitude = error;
	else if(error < minAmplitude)
	    minAmplitude = error;
    }

    // Extracts the time between zero crossings
    if( (prevError >  0.0 && error <= 0.0) ||
	(prevError <= 0.0 && error >  0.0) )
    {
	firstZeroCrossing = true;
	outputTimePeriodSec = 2.0*(deltaOutputTime + Ts);
//	cout << " Zero crossing " << currTime 
//	     << " delta time " << deltaOutputTime 
//	     << " outputTimePeriodSec " << outputTimePeriodSec << endl; 
	deltaOutputTime = 0.0;
    }
    else
	deltaOutputTime += Ts;
    prevError = error;

//    cout << currTime << " "
//	 << inputAmplitude << " " 
//	 << _measuredValue << endl;


    // Stops after test time period
    currTime += Ts;
    if(currTime <= testTimeSec) 
    {
	return ((error) / fabs(error)) * inputAmplitude; 
    }
    else 
    {
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

    cout << " Kcu " << ultimateGain 
	 << " Tc " << outputTimePeriodSec 
         << " A " << 	outputAmplitude
         << " Kp " << _Kp 
	 << " Ti " << _Ti 
	 << " Td " << _Td << endl;
}
