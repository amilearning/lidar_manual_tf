cmake_minimum_required(VERSION 3.0.2)
project(dynamic_reconfigure_test)

find_package(catkin REQUIRED COMPONENTS
     roscpp
     rospy
     dynamic_reconfigure
     std_msgs
     message_generation

)

find_package(Eigen3)


add_message_files(
  FILES
  euler.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
     cfg/test.cfg
)

catkin_package(
     INCLUDE_DIRS include
     LIBRARIES dynamic_reconfigure_test
     CATKIN_DEPENDS roscpp rospy dynamic_reconfigure 
     DEPENDS system_lib
)

include_directories(
     ${catkin_INCLUDE_DIRS}
     ${EIGEN3_INCLUDE_DIR}

)

add_executable(dynamic_reconfigure_test_exe src/dynamic_reconfigure_test.cpp)

target_link_libraries(dynamic_reconfigure_test_exe  ${catkin_LIBRARIES})

add_dependencies(dynamic_reconfigure_test_exe ${PROJECT_NAME}_gencfg)

# add_executable(dynamic_reconfigure_test/dynamic_reconfigure_test.cpp)
# add_dependencies(dynamic_reconfigure_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(dynamic_reconfigure_test ${catkin_LIBRARIES})

add_executable(msg_sub src/msg_sub.cpp)
add_dependencies(msg_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(msg_sub ${catkin_LIBRARIES})



