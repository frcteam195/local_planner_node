cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME "local_planner_node")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(${PROJECT_NAME})

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rio_control_node teb_local_planner)

add_message_files(
  FILES
  TrajectoryFollowCue.msg
  LocalPlannerDiagnostics.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  PlanReq.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rio_control_node
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
  CATKIN_DEPENDS roscpp message_generation message_runtime rio_control_node
)
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CMAKE_SOURCE_DIR}/../../../third_party_libs/ROSProtoDef2022/out
)

message("CATKIN INCLUDES:${catkin_INCLUDE_DIRS}")

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)

file(GLOB_RECURSE executable_sources CONFIGURE_DEPENDS
     "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

file(GLOB_RECURSE test_sources CONFIGURE_DEPENDS
     "${PROJECT_SOURCE_DIR}/test/src/*.cpp"
)

add_executable(${PROJECT_NAME} ${executable_sources})
target_include_directories(${PROJECT_NAME} PUBLIC "include")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Threads::Threads)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  ## Add gtest based cpp test target and link libraries.
  add_library(t${PROJECT_NAME} ${executable_sources})
  target_include_directories(t${PROJECT_NAME} PUBLIC "include")
  target_link_libraries(t${PROJECT_NAME} ${catkin_LIBRARIES} Threads::Threads)
  add_dependencies(t${PROJECT_NAME} ${PROJECT_NAME} ${t${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  catkin_add_gtest(${PROJECT_NAME}-test ${test_sources})
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test t${PROJECT_NAME})
    target_include_directories(${PROJECT_NAME}-test PUBLIC "include" "test/include")
    add_dependencies(${PROJECT_NAME}-test t${PROJECT_NAME} ${t${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  endif()
endif()
