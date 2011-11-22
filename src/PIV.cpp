//
// C++ Implementation: PIV
//
// Description:
//
//
// Author:  <Ajish Babu>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "PIV.hpp"
#include <iostream>

using namespace motor_controller;

PIV::PIV()
{
    setGains ( 0, 0, 0 );
    setFeedForwardGain(0, 0);
    setVelSmoothingGain( 0 );
    setSamplingTime ( 0 );
    setOutputLimits ( 0, 0 );
    setIntegratorWindupCoeff ( 0 );
    setPositionController(true);
    reset();
}

PIV::PIV ( 
	double _Kpp, double _Kiv, double _Kpv, 
	double _Kvff, double _Kaff,
	double _Kalp,
	double _Ts,
	double _YMin, double _YMax, 
	double _Kt)
{
    setGains ( _Kpp, _Kiv,  _Kpv );
    setFeedForwardGain( _Kvff, _Kaff );
    setVelSmoothingGain( _Kalp );
    setSamplingTime ( _Ts );
    setOutputLimits ( _YMin,  _YMax );
    setIntegratorWindupCoeff ( _Kt );
    setPositionController(true);
    reset();
}
        
PIV::PIV ( const struct PIVSettings& _settings ) 
{
    setPIVSettings( _settings );
    setPositionController(true);
}

PIV::~PIV()
{
}

    void
PIV::setPIVSettings ( const struct PIVSettings& _settings )
{
    setGains ( _settings.Kpp, _settings.Kiv,  _settings.Kpv );
    setFeedForwardGain( _settings.Kvff, _settings.Kaff );
    setVelSmoothingGain( _settings.Kalp );
    setSamplingTime ( _settings.Ts );
    setOutputLimits ( _settings.YMin,  _settings.YMax );
    setIntegratorWindupCoeff ( _settings.Kt );
    reset();
}

	void 
PIV::setGains ( double _Kpp, double _Kiv, double _Kpv )
{
    Kpp = _Kpp;
    Kiv = _Kiv;
    Kpv = _Kpv;
}

	void 
PIV::setSamplingTime ( double _Ts )
{
    Ts = _Ts;
    velITerm.init ( Ts );
}

	double 
PIV::saturate_windup ( double _val )
{
    if ( _val > YMax )
    {
	limitDiff = YMax - _val;
	return YMax;
    }
    else if ( _val < YMin )
    {
	limitDiff = YMin - _val;
	return YMin;
    }
    else 
	return _val;
}

	double 
PIV::saturate ( double _val )
{
    if ( _val > YMax )
	return YMax;
    else if ( _val < YMin )
	return YMin;
    else 
	return _val;
}

	double 
PIV::updateVelLoop ( double _velMeasured, double _velCmd, double _posCommand, double _accFF )
{
    velSmooth = (1-Kalp)*_velMeasured + Kalp*velPrevStep;
    velPrevStep = velSmooth;
    velCommand = (Kvff * _velCmd) + _posCommand - velSmooth;
    velCommand = saturate_windup((Kpv*velCommand) + velITerm.update(Kiv*velCommand + Kt*limitDiff));
    velCommand = saturate(velCommand + Kaff * _accFF); 
    return velCommand;
}

	double 
PIV::update ( double _posMeasured, double _posRef, double _velFF, double _accFF )
{
    if(firstRun)
    {
	posPrevStep = _posMeasured;
	velPrevStep = 0.0;
	firstRun = false;
    }

    velComputed = (_posMeasured - posPrevStep ) / Ts;
    if(posController)
	posCommand = updatePosLoop(_posRef - _posMeasured);
    else 
	posCommand = 0.0;

    posPrevStep = _posMeasured;
    return updateVelLoop(velComputed, _velFF, posCommand, _accFF);
}

	void 
PIV::reset()
{
    limitDiff = 0.0;
    velPrevStep = 0.0;
    velITerm.init(Ts);
    firstRun = true;
}

        void
PIV::printCoefficients()
{
    std::cout << "PIV Coefficients : " << std::endl ;
    std::cout << " Kpp : " << Kpp << std::endl 
              << " Kiv : " << Kiv << std::endl 
              << " Kpv : " << Kpv << std::endl 
              << " Kvff : " << Kvff << std::endl 
              << " Kaff : " << Kaff << std::endl 
              << " Kalp : " << Kalp << std::endl 
              << " Kt : "   << Kt << std::endl 
              << " Ts : " << Ts << std::endl 
              << " YMax : " << YMax << std::endl 
              << " YMin : " << YMin << std::endl ;
}
