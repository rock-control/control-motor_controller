
PROJECT(motor_controller)

cmake_minimum_required(VERSION 2.6)

#add definitions, compiler switches, etc.
ADD_DEFINITIONS(-Wall)

#list all source files here
#ADD_EXECUTABLE(driver main.cpp interface.cpp)
find_package(PkgConfig)

pkg_check_modules(BASE_TYPES REQUIRED "base-types")
INCLUDE_DIRECTORIES(${BASE_TYPES_INCLUDE_DIRS})
LINK_DIRECTORIES(${BASE_TYPES_LIBRARY_DIRS})

INCLUDE_DIRECTORIES(/usr/include/eigen2)

ADD_LIBRARY(motor_controller SHARED 
	    PIV.cpp
	    RK4Integrator.cpp
	    SimpleIntegrator.cpp
	    Ramp.cpp
	    Ramp.hpp
	    PIV.hpp
	    RK4Integrator.hpp
	    SimpleIntegrator.hpp)
 
CONFIGURE_FILE(Doxyfile.in Doxyfile @ONLY)
ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
INSTALL(TARGETS motor_controller
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
INSTALL(FILES  
	Ramp.hpp
       	PIV.hpp
        RK4Integrator.hpp
        SimpleIntegrator.hpp
	DESTINATION include)

CONFIGURE_FILE(motor_controller.pc.in motor_controller.pc @ONLY)
INSTALL(FILES ${CMAKE_BINARY_DIR}/motor_controller.pc DESTINATION lib/pkgconfig)
