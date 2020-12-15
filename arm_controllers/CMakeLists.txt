cmake_minimum_required(VERSION 2.8.3)
project(arm_controllers)

find_package(catkin REQUIRED COMPONENTS
  angles
  controller_interface
  control_msgs
  urdf
  kdl_parser
  realtime_tools
  dynamic_reconfigure
)

# Dynamics reconfigure
generate_dynamic_reconfigure_options(
  cfg/PassivityControllerParams.cfg
  )

# message
add_message_files(FILES ControllerJointState.msg)
generate_messages(DEPENDENCIES std_msgs)

# include
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    angles
    controller_interface
    control_msgs
    urdf
    kdl_parser
    realtime_tools
    dynamic_reconfigure
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/time_delay_controller.cpp
  src/gravity_comp_controller.cpp
  src/computed_torque_controller.cpp
  src/custom_torque_controller.cpp
  src/computed_torque_clik_controller.cpp
  src/adaptive_variable_impedance_controller.cpp
  src/passivity_controller.cpp
  src/task_space_controller.cpp
  src/visual_servo_controller.cpp
  src/gravity_controller_reactive.cpp
  src/demo_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})





install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


catkin_install_python(PROGRAMS scripts/safety_test.py scripts/publish_test.py scripts/task_space_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


