//
// C++ Implementation: PositionVelocityTorqueCascade
//
// Description:
//
//
// Author:  <Ajish Babu>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "PositionVelocityTorqueCascade.hpp"

using namespace motor_controller;

PositionVelocityTorqueCascade::PositionVelocityTorqueCascade()
{
    setSamplingTime ( 0 );
    setGains ( 0, 0, 0, 0, 0 );
    setFeedForwardGain(0, 0);
    setVelSmoothingGain( 0 );
    setOutputLimits ( 0, 0 );
    setIntegratorWindupCoeffs ( 0, 0 );
    setPositionController(true);
    setVelocityController(true);

    velPrevStep = 0.0;
    firstRun = true;
}

PositionVelocityTorqueCascade::PositionVelocityTorqueCascade ( 
	double _Ts,
	double _Kpp, 
	double _Kpv, double _Kiv, 
	double _Kpt, double _Kit, 
	double _Kalp,
	double _Ktv, double _Ktt,
	double _Kvff, double _Kaff,
	double _YMin, double _YMax)
{
    setSamplingTime ( _Ts );
    setGains ( _Kpp, _Kpv, _Kiv, _Kpt, _Kit );
    setFeedForwardGain( _Kvff, _Kaff );
    setVelSmoothingGain( _Kalp );
    setOutputLimits ( _YMin,  _YMax );
    setIntegratorWindupCoeffs ( _Ktv, _Ktt );

    setPositionController(true);
    setVelocityController(true);

    velPrevStep = 0.0;
    firstRun = true;
}

PositionVelocityTorqueCascade::~PositionVelocityTorqueCascade()
{
}

	void 
PositionVelocityTorqueCascade::setSamplingTime ( double _Ts )
{
    Ts = _Ts;
    setGains();
}
	    
	void 
PositionVelocityTorqueCascade::setFeedForwardGain( double _Kvff, double _Kaff )
{ 
    Kvff = _Kvff; 
    Kaff = _Kaff; 
    setGains();
};
	    
	void 
PositionVelocityTorqueCascade::setVelSmoothingGain( double _Kalp )  
{ 
    Kalp = _Kalp; 
    setGains();
};

	void 
PositionVelocityTorqueCascade::setOutputLimits ( double _YMin, double _YMax )
{ 
    YMin = _YMin; 
    YMax = _YMax; 
    setGains();
};

        void 
PositionVelocityTorqueCascade::setIntegratorWindupCoeffs (double _Ktv, double _Ktt) 
{ 
    Ktv = _Ktv; 
    Ktt = _Ktt; 
    setGains();
};

	void 
PositionVelocityTorqueCascade::setGains (double _Kpp,
		    double _Kpv, double _Kiv,
		    double _Kpt, double _Kit )
{
    Kpp = _Kpp;
    Kpv = _Kpv;    Kiv = _Kiv;
    Kpt = _Kpt;    Kit = _Kit;
    setGains();
}

	void 
PositionVelocityTorqueCascade::setGains()
{
    PositionP.setParallelCoefficients(Ts, Kpv );
    VelocityPI.setParallelCoefficients(Ts, Kpv, Kiv, 0, 0, 0, Ktv*Ts );
    TorquePI.setParallelCoefficients(Ts, Kpt, Kit, 0, 0, 0, Ktt*Ts, YMin, YMax );
}

	double 
PositionVelocityTorqueCascade::update ( double _posMeasured, double _posRef, double _torMeasured, double _velFF, double _accFF )
{
    if(firstRun)
    {
	posPrevStep = _posMeasured;
	velPrevStep = 0.0;
	firstRun = false;
    }

    velComputed = (_posMeasured - posPrevStep ) / Ts;
    posPrevStep = _posMeasured;
    velSmooth = (1-Kalp)*velComputed + Kalp*velPrevStep;
    velPrevStep = velSmooth;

    if(velController)
    {
	if(posController)
	    velCommand = PositionP.update(_posMeasured, _posRef);
	else 
	    velCommand = 0.0;
	torCommand = VelocityPI.update(velSmooth, velCommand + Kvff * _velFF);
    }
    else 
    {
	torCommand = 0.0;
    }
    
    torCommand = TorquePI.update(_torMeasured, torCommand + Kaff * _accFF);
    return torCommand;
}

	void 
PositionVelocityTorqueCascade::reset()
{
    PositionP.reset();
    VelocityPI.reset();
    TorquePI.reset();

    velPrevStep = 0.0;
    firstRun = true;
}

        void
PositionVelocityTorqueCascade::printCoefficients()
{
//    std::cout << "PVT Cascade Coefficients : " << std::endl ;
//    std::cout << " Kpp : " << Kpp << std::endl 
//              << " Kiv : " << Kiv << std::endl 
//              << " Kpv : " << Kpv << std::endl 
//              << " Kvff : " << Kvff << std::endl 
//              << " Kaff : " << Kaff << std::endl 
//              << " Kalp : " << Kalp << std::endl 
//              << " Kt : "   << Kt << std::endl 
//              << " Ts : " << Ts << std::endl 
//              << " YMax : " << YMax << std::endl 
//              << " YMin : " << YMin << std::endl ;
}
