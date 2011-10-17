//
// C++ Implementation: InputShaper
//
// Description:
//
//
// Author:  <Ajish Babu>, (C) 2011
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "InputShaper.hpp"

using namespace motor_controller;
using namespace std;

	void 
InputShaper::setCoefficients (double _Ts, 
		    double _dampingRatio,
		    double _naturalFrequency,
		    int _nrImpulses)
{
    Ts = _Ts;
    dampingRatio = _dampingRatio;
    naturalFrequency = _naturalFrequency;
    nrImpulses = _nrImpulses;

    if(nrImpulses < 2)
    {
	cout << "INPUT SHAPER : Nr of impulses less than minimum allowed, so will be set to 2" << endl;
	nrImpulses = 2;
    }
    else if(nrImpulses > 4)
    {
	cout << "INPUT SHAPER : Nr of impulses more than maximum allowed, so will be set to 4" << endl;
	nrImpulses = 4;
    }

    K = exp(- dampingRatio*M_PI / sqrt(1 - pow(dampingRatio, 2)));
    timeDelay = M_PI / naturalFrequency / sqrt(1 - pow(dampingRatio, 2));

    amplitudes.resize(nrImpulses);

    if(nrImpulses == 2)
    {
	double denominator = 1+K;
	amplitudes.at(0) = 1.0 / denominator; 
	amplitudes.at(1) = K   / denominator; 
    }
    else if(nrImpulses == 3)
    {
	double denominator = (1+K)*(1+K);
	amplitudes.at(0) = 1.0   / denominator;
	amplitudes.at(1) = 2.0*K / denominator; 
	amplitudes.at(2) = K*K   / denominator; 
    }
    else if(nrImpulses == 4)
    {
	double denominator = (1+K)*(1+K)*(1+K);
	amplitudes.at(0) = 1.0 	   / denominator; 
	amplitudes.at(1) = 3.0*K   / denominator; 
	amplitudes.at(2) = 3.0*K*K / denominator; 
	amplitudes.at(3) = K*K*K   / denominator; 
    }

    reset();
}

	void 
InputShaper::setDirectCoefficients (double _Ts, 
		    double _K,
		    double _timeDelay,
		    int _nrImpulses)
{
    Ts = _Ts;
    K = _K;
    timeDelay = _timeDelay;
    nrImpulses = _nrImpulses;

    if(nrImpulses < 2)
    {
	cout << "INPUT SHAPER : Nr of impulses less than minimum allowed, so will be set to 2" << endl;
	nrImpulses = 2;
    }
    else if(nrImpulses > 4)
    {
	cout << "INPUT SHAPER : Nr of impulses more than maximum allowed, so will be set to 4" << endl;
	nrImpulses = 4;
    }

    amplitudes.resize(nrImpulses);

    if(nrImpulses == 2)
    {
	double denominator = 1+K;
	amplitudes.at(0) = 1.0 / denominator; 
	amplitudes.at(1) = K   / denominator; 
    }
    else if(nrImpulses == 3)
    {
	double denominator = (1+K)*(1+K);
	amplitudes.at(0) = 1.0   / denominator;
	amplitudes.at(1) = 2.0*K / denominator; 
	amplitudes.at(2) = K*K   / denominator; 
    }
    else if(nrImpulses == 4)
    {
	double denominator = (1+K)*(1+K)*(1+K);
	amplitudes.at(0) = 1.0 	   / denominator; 
	amplitudes.at(1) = 3.0*K   / denominator; 
	amplitudes.at(2) = 3.0*K*K / denominator; 
	amplitudes.at(3) = K*K*K   / denominator; 
    }

    reset();
}

    	void
InputShaper::printCoefficients()
{ 
    cout << "INPUT SHAPER : " << endl;
    cout << "  Nr of impulses = " << nrImpulses << endl;
    cout << "  Damping Ratio = " << dampingRatio << endl;
    cout << "  Natural Frequency = " << naturalFrequency << endl;

    cout << "  Amplitudes = [";
    for(int i=0; i < nrImpulses; i++)
	cout << amplitudes.at(i) << "," ;

    cout << "  Time delay = " << timeDelay << endl;
}

	double 
InputShaper::update ( double _referenceInput, 
	double time )
{
    currTime += Ts;
    shapedCommand = 0; 

    referenceInput.push_front(_referenceInput);

    // Adds all the impulse amplitudes upto the current time
    for(int i=0; i < nrImpulses; i++)
    {
	// delay the impulse
	if(currTime >= i * timeDelay + Ts )
	{
/*	    cout << currTime  
		<< " i:" << i 
		<< " a:" << amplitudes.at(i) 
		<< " size:" << referenceInput.size() ;
*/
	    shapedCommand +=  amplitudes.at(i) * referenceInput.at(i * (timeDelay / Ts));
/*
	    cout << " r:" << referenceInput.at(i * (timeDelay / Ts));
	    cout << " s:" << shapedCommand
		<< " index:" << i * (timeDelay / Ts)
		<< endl;
*/
	}
    }

    if( referenceInput.size() >  (nrImpulses-1)*timeDelay / Ts )
	referenceInput.pop_back();

    return shapedCommand;
}
