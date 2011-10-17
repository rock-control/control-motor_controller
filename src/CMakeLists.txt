rock_library(motor_controller
    SOURCES InputShaper.cpp PID.cpp PIV.cpp Ramp.cpp RK4Integrator.cpp SimpleIntegrator.cpp
    HEADERS InputShaper.hpp PID.hpp PIV.hpp Ramp.hpp RK4Integrator.hpp
    SimpleIntegrator.hpp)

# For backward compatibility only
install(FILES  
	compat/Ramp.hpp
       	compat/PIV.hpp
	compat/PID.hpp
	compat/InputShaper.hpp
        compat/RK4Integrator.hpp
        compat/SimpleIntegrator.hpp
	DESTINATION include)

