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
	frc_robot_interface
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
	set (WPI_LIB_DIR  $ENV{HOME}/wpilib/cpp/current/lib/linux/athena/shared)
	set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L $ENV{HOME}/allwpilib/build/integrationTestFiles/libs")
	target_link_libraries(frcrobot_hw_main
	  ${catkin_LIBRARIES}
	  frc_robot_interface
	  generic_hw_control_loop
	  ${CTRE_LIB_DIR}/libCTRLibDriver.a
	  ${CTRE_LIB_DIR}/libCTRLib.a
	  ${WPI_LIB_DIR}/libFRC_NetworkCommunication.so.18.0.0
	  ${WPI_LIB_DIR}/libNiFpgaLv.so.17.0.0
	  ${WPI_LIB_DIR}/libNiFpga.so.17.0.0
	  ${WPI_LIB_DIR}/libniriodevenum.so.17.0.0
	  ${WPI_LIB_DIR}/libniriosession.so.17.0.0
	  ${WPI_LIB_DIR}/libNiRioSrv.so.17.0.0
	  ${WPI_LIB_DIR}/libRoboRIO_FRC_ChipObject.so.18.0.0
	  ${WPI_LIB_DIR}/libspi.so.1.0.0
	  ${WPI_LIB_DIR}/libwpiHal.so
	  ${WPI_LIB_DIR}/libwpilibc.so
	  ${WPI_LIB_DIR}/libwpi.so
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
