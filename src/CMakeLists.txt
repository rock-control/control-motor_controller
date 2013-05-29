rock_library(motor_controller
    SOURCES InputShaper.cpp PID.cpp PIV.cpp Ramp.cpp RK4Integrator.cpp
    SimpleIntegrator.cpp PositionVelocityTorqueCascade.cpp IRSpeedFromPositionFilter.cpp
    HEADERS InputShaper.hpp PID.hpp PIV.hpp Ramp.hpp RK4Integrator.hpp
    SimpleIntegrator.hpp PositionVelocityTorqueCascade.hpp IRSpeedFromPositionFilter.hpp
    DEPS_PKGCONFIG base-types)

# For backward compatibility only
install(FILES  
	compat/Ramp.hpp
       	compat/PIV.hpp
	compat/PID.hpp
	compat/InputShaper.hpp
        compat/RK4Integrator.hpp
        compat/SimpleIntegrator.hpp
        compat/PositionVelocityTorqueCascade.hpp
	DESTINATION include)

