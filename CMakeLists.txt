cmake_minimum_required(VERSION 3.1)

project(scriptsculpt)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(MAYA_LOCATION "C:\\Program Files\\Autodesk\\Maya2024")
set(MAYA_PLUGIN_OUTPUT_DIR "C:\\Users\\2000\\devkitBase\\plug-ins\\plug-ins")

set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
########################################
# EIGEN Setup
########################################
include(FetchContent)
FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE)
set(EIGEN_BUILD_DOC OFF)

set(BUILD_TESTING OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
FetchContent_MakeAvailable(Eigen)
########### end ##########################

# Enable C++11 for host code
set(CMAKE_CXX_STANDARD 11)

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

########################################
# OPENCV Setup
########################################
FIND_PACKAGE( OpenCV REQUIRED )                              
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )       
########### end ##########################

find_package(Maya REQUIRED)

###########################################

file(GLOB SOURCE_FILES ${SRC_DIR}/*.h
    ${SRC_DIR}/*.cpp)

############################################
#set(headers
#    src/utilities.h
#    src/ConstLetter.h
#    src/sceneStruct.h
#    src/LetterAlignment.h
#    src/openGA.hpp
#    src/GASolver.h
#    src/deformation.h
#    src/ScriptSculptCommand.h
#    )

#set(sources
#    #src/main.cpp
#    src/PluginMain.cpp
#    src/utilities.cpp
#    src/ConstLetter.cpp
#    src/trace_skeleton.cpp
#    src/LetterAlignment.cpp
#    src/GASolver.cpp
#    src/deformation.cpp
#    src/ScriptSculptCommand.cpp
#    )
#
#list(SORT headers)
#Alist(SORT sources)
#
#source_group(Headers FILES ${headers})
#source_group(Sources FILES ${sources})
#
#add_executable(${CMAKE_PROJECT_NAME} ${sources} ${headers})
###################################
add_library(${CMAKE_PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${CMAKE_PROJECT_NAME}
    ${OpenCV_LIBS}
    ${LIBRARIES}
    Eigen3::Eigen
    Maya::Maya
    )
MAYA_PLUGIN(${CMAKE_PROJECT_NAME})
set(PLUGIN_OUTPUT_PATH ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.mll)
if(EXISTS ${PLUGIN_OUTPUT_PATH})
	configure_file(${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.mll ${MAYA_PLUGIN_OUTPUT_DIR} COPYONLY)
endif()