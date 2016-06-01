cmake_minimum_required(VERSION 2.8.3)
project(person_recognizer)

#Debug Option (Commentout to Enable)#
set(CMAKE_BUILD_TYPE Debug)

### Works on C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nodelet
  pluginlib
  sensor_msgs
  std_msgs
  roscpp
  rospy
  message_generation
)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#Run cppcheck#                                                                                                                                                                                                                    
execute_process(
  COMMAND "cppcheck" "--enable=all" "src" "-I" "include" "--suppress=missingIncludeSystem" "|" "tee" "logs/cppcheck.log"
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)
#Run cpplint#                                                                                                                                                                                                                     
execute_process(
  COMMAND "cpplint" "--recursive" "src" "include" "|" "tee" "logs/cpplint.log"
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

#Delete Past Docs#                                                                                                                                                                                                                
execute_process(
  COMMAND "rm" "-rf" "docs"
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

#Generate Docs#                                                                                                                                                                                                                   
execute_process(
  COMMAND "doxygen"
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

#Make Docs dir#                                                                                                                                                                                                                   
execute_process(
  COMMAND "mkdir" "docs"
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}

)

#Move Files to docs dir#                                                                                                                                                                                                          
execute_process(
  COMMAND "mv" "html" "latex" "docs"
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Face.msg
#   Message1.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs#   std_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES facedetector_nodelet
  CATKIN_DEPENDS cv_bridge image_transport nodelet pluginlib sensor_msgs std_msgs rospy roscpp message_runtime
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  include/
  opencv/include
  opencv/build/modules/
)

link_directories(
  ${PROJECT_SOURCE_DIR}/opencv/build/lib
)

## Declare a C++ library
# add_library(person_recognizer
#   src/${PROJECT_NAME}/person_recognizer.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(person_recognizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(person_recognizer_node src/person_recognizer_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(person_recognizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(person_recognizer_node
#   ${catkin_LIBRARIES}
# )
add_executable(face_detector src/facedetector_node.cpp)

add_library(FaceDetector src/FaceDetector.cpp)
add_library(facedetector_nodelet src/facedetector_nodelet.cpp)
add_dependencies(facedetector_nodelet FaceDetector)

target_link_libraries(face_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CUDA_CUDA_LIBRARY}
  ${CUDA_CUDART_LIBRARY}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS face_detector facedetector_nodelet FaceDetector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/{$PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES personrecognizer_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS person_recognizer person_recognizer_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_person_recognizer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
