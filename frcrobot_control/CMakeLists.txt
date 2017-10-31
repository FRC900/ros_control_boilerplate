include_directories(include)

#add_definitions(-Wno-unused-parameter -Wno-return-type -Wno-ignored-qualifiers)

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
    # should be built for the RoboRIO since
    # it direcly interfaces with the RIO
    # hardware & attached devices

    # Main control executable
	add_executable(frcrobot_hw_main 
		src/frcrobot_hw_main.cpp
		src/frcrobot_hw_interface.cpp
	)

    # Switch between 2017 and 2018 beta wpilib code
    if (0)
	  include_directories(
	    $ENV{HOME}/wpilib_2017/user/cpp/include
	    $ENV{HOME}/wpilib_2017/cpp/current/include
	  )
	  
	  set (CTRE_LIB_DIR $ENV{HOME}/wpilib_2017/user/cpp/lib)
	  set (CTRE_LIBS
	  	  ${CTRE_LIB_DIR}/libCTRLibDriver.a
		  ${CTRE_LIB_DIR}/libCTRLib.a
	  )

	else() # 2018 setup
	  include_directories(
	    $ENV{HOME}/wpilib/user/cpp/include
	    $ENV{HOME}/wpilib/cpp/current/include
	  )
	  
	  set (WPI_LIB_DIR         $ENV{HOME}/wpilib/cpp/current/reflib/linux/athena/shared)
	  set (WPI_COMMON_LIB_DIR  $ENV{HOME}/wpilib/common/current/lib/linux/athena/shared)
	  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L ${WPI_LIB_DIR}")
	  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L ${WPI_COMMON_LIB_DIR}")

	  set (CTRE_LIB_DIR $ENV{HOME}/wpilib/user/cpp/lib)
	  set (CTRE_LIBS
		  ${CTRE_LIB_DIR}/libCTRE_PhoenixCCI.a
		  ${CTRE_LIB_DIR}/libCTRE_Phoenix.a
	  )
    endif()

	# Link against CTRE for CAN talon code and wpilib
	# for HAL, driver station, and so on
	target_link_libraries(frcrobot_hw_main
		${catkin_LIBRARIES}
		frc_robot_interface
		generic_hw_control_loop
		${WPI_LIB_DIR}/libwpi.so
		${WPI_LIB_DIR}/libFRC_NetworkCommunication.so.18.0.0
		${CTRE_LIBS}
	)

    ## Install ------------------------------------------------------------

    # Install executables
	install(TARGETS
	  frcrobot_hw_main
	  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
endif()

	install(TARGETS
	  frcrobot_sim_main
	  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
# Install header files
install(DIRECTORY 
	include/frcrobot_control
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY
	launch
	config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/frcrobot_control
)
