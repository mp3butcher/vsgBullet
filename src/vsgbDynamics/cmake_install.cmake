# Install script for directory: /media/xeul/BIGEXT4/SRC/osgbullet-master/src/osgbDynamics

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "libosgbbullet-dev" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/media/xeul/BIGEXT4/SRC/osgbullet-master/lib/libosgbDynamics.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "libosgbbullet-dev" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/osgbDynamics" TYPE FILE FILES
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbDynamics/Export.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbDynamics/Constraints.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbDynamics/CreationRecord.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbDynamics/GroundPlane.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbDynamics/MotionState.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbDynamics/PhysicsThread.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbDynamics/RigidBody.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbDynamics/Joint.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbDynamics/SoftBody.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbDynamics/RigidBodyAnimation.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbDynamics/TripleBuffer.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbDynamics/World.h"
    )
endif()

