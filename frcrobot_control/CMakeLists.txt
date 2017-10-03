include_directories(
  include/
)

# FRCRobot Hardware Interface
add_library(frcrobot_hw_interface
  src/frcrobot_hw_interface.cpp
)
target_link_libraries(frcrobot_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(frcrobot_hw_main src/frcrobot_hw_main.cpp)
target_link_libraries(frcrobot_hw_main
  frcrobot_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    frcrobot_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    frcrobot_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/frcrobot_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
