# Install script for directory: /media/xeul/BIGEXT4/SRC/osgbullet-master/src/osgbCollision

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/media/xeul/BIGEXT4/SRC/osgbullet-master/lib/libosgbCollision.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "libosgbbullet-dev" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/osgbCollision" TYPE FILE FILES
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/BoundingCone.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/BoundingCylinder.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/Chart.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/CollectVerticesVisitor.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/CollisionShapes.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/ComputeCylinderVisitor.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/ComputeShapeVisitor.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/ComputeTriMeshVisitor.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/GLDebugDrawer.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/RefBulletObject.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/Utils.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/Version.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/VertexAggOp.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/Export.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/AbsoluteModelTransform.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/GeometryOperation.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/ReducerOp.h"
    "/media/xeul/BIGEXT4/SRC/osgbullet-master/include/osgbCollision/GeometryModifier.h"
    )
endif()

