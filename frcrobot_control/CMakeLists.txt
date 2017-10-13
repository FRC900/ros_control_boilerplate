include_directories(
  include/
)

# Sim version of main control
# executable.  For now this just prints debugging
# info for testing code which doens't actually hit
# RoboRIO or CAN hardware. Eventually expand into
# controlling some sort of sim?
add_executable(frcrobot_sim_main 
	src/frcrobot_sim_main.cpp
	src/frcrobot_sim_interface.cpp
	)
target_link_libraries(frcrobot_sim_main
	generic_hw_interface
	generic_hw_control_loop
	${catkin_LIBRARIES}
)

if (DEFINED CMAKE_TOOLCHAIN_FILE)
    # FRCRobot Hardware Interface only
    # should be build for the RoboRIO since
    # it direcly interfaces with the RIO
    # hardware & attached devices

    # Main control executable
	add_executable(frcrobot_hw_main 
		src/frcrobot_hw_main.cpp
		src/frcrobot_hw_interface.cpp
	)
	include_directories(
	  $ENV{HOME}/wpilib/user/cpp/include
	  $ENV{HOME}/wpilib/cpp/current/include
	)
	  
	set (CTRE_LIB_DIR $ENV{HOME}/wpilib/user/cpp/lib)
	set (WPI_LIB_DIR  $ENV{HOME}/wpilib/cpp/current/lib)
	target_link_libraries(frcrobot_hw_main
	  ${catkin_LIBRARIES}
	  generic_hw_interface
	  generic_hw_control_loop
	  ${CTRE_LIB_DIR}/libCTRLibDriver.a
	  ${CTRE_LIB_DIR}/libCTRLib.a
	  ${WPI_LIB_DIR}/libnirio_emb_can.so
	  ${WPI_LIB_DIR}/libni_emb.so
	  ${WPI_LIB_DIR}/libHALAthena.so
	  ${WPI_LIB_DIR}/libi2c.so
	  ${WPI_LIB_DIR}/libRoboRIO_FRC_ChipObject.so
	  ${WPI_LIB_DIR}/libspi.so
	  ${WPI_LIB_DIR}/libNiFpga.so
	  ${WPI_LIB_DIR}/libNiFpgaLv.so
	  ${WPI_LIB_DIR}/libFRC_NetworkCommunication.so
	  ${WPI_LIB_DIR}/libNiRioSrv.so
	  ${WPI_LIB_DIR}/libniriodevenum.so
	  ${WPI_LIB_DIR}/libniriosession.so
	  ${WPI_LIB_DIR}/libni_rtlog.so
	  ${WPI_LIB_DIR}/libwpi.so
	  ${WPI_LIB_DIR}/libwpilibc.so
	)

    ## Install ------------------------------------------------------------

    # Install executables
	install(TARGETS
		frcrobot_hw_main
	  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
endif()

# Install header files
install(DIRECTORY include/frcrobot_control/  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY
	launch
	config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/frcrobot_control
)
