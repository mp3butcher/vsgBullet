# Install script for directory: /media/xeul/BIGEXT4/SRC/osgbullet-master/data

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "libosgworks" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vsgBullet/data" TYPE FILE FILES
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/GateWall-hinge.txt"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/GateWall.flt"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/GateWall.jpg"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/NightStand.flt"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/NightStand.jpg"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/ShadowMap-Main-3x.fs"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/ShadowMap-Main.fs"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/block.osg"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/com.osg"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/compound.osg"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/concave.osg"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/concave.txt"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/concavesg.osg"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/constraint.osg"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/constraint.txt"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/constraintsg.osg"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/dice.osg"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/fort_mchenry_flag.jpg"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/hand.osg"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/hand.png"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/hud.fs"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/hud.vs"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/offcube.osg"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/saverestore-scene.osg"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/tetra.osg"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/data/tex_dice.png"
    )
endif()

