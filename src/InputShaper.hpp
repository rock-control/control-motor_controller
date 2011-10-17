//
// C++ Interface: InputShaper 
//
// Description: Input Shaper is command generation technique used to remove the vibrations from 
// the system with known dynamics
//
//
// Author:  <Ajish Babu>, (C) 2011
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef CONTROLLERINPUTSHAPERCONTROLLER_H
#define CONTROLLERINPUTSHAPERCONTROLLER_H

#include <math.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <deque>

namespace motor_controller
{
    class InputShaper
    {
	public:
	    InputShaper() {};
	    void setCoefficients(double _Ts, 
		    double _dampingRatio,
		    double _naturalFrequency,
		    int nrImpulses = 2);

	    void setDirectCoefficients (double _Ts, 
		    double _K,
		    double _timeDelay,
		    int _nrImpulses = 2);

	    void printCoefficients();

	    double update ( double _referenceInput, 
		    double time = 0.0  );

	    void reset() { currTime = 0.0; 
		referenceInput.clear();}

	private:

	    double Ts; // Sampling time
	    double currTime; // Integrated sampling time

	    int nrImpulses;  // Number of modes for the input shaper
	    	    	     // higher the number of modes .... higher the robustness
			     // 2 <= nrImpulses <= 4
			     // needs to be extended for further modes if needed
	    double dampingRatio; // Damping ratio of the system 
	    double naturalFrequency; // Natural frequency of the system \omega_0 

	    std::vector<double> amplitudes; // The impulse amplitudes
	    double K; // Step response overshoot
	    double timeDelay; // Time of the first overshoot 

	    double shapedCommand; // the shaped command output

	    std::deque<double> referenceInput; // Stores the reference input for delayed signal
    };
}

#endif
