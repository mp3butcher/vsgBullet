# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#[=======================================================================[.rst:
FindBullet
----------

Try to find the Bullet physics engine



::

  This module defines the following variables



::

  BULLET_FOUND - Was bullet found
  BULLET_INCLUDE_DIRS - the Bullet include directories
  BULLET_LIBRARIES - Link to this, by default it includes
                     all bullet components (Dynamics,
                     Collision, LinearMath, & SoftBody)



::

  This module accepts the following variables



::

  BULLET_ROOT - Can be set to bullet install path or Windows build path
#]=======================================================================]

macro(_FIND_BULLET_LIBRARY _var)
  find_library(${_var}
     NAMES
        ${ARGN}
     HINTS
        ${BULLET_ROOT}
        ${BULLET_ROOT}/lib/Release
        ${BULLET_ROOT}/lib/Debug
        ${BULLET_ROOT}/out/release8/libs
        ${BULLET_ROOT}/out/debug8/libs
     PATH_SUFFIXES lib
  )
  mark_as_advanced(${_var})
endmacro()

macro(_BULLET_APPEND_LIBRARIES _list _release)
  set(_debug ${_release}_DEBUG)
  if(${_debug})
    set(${_list} ${${_list}} optimized ${${_release}} debug ${${_debug}})
  else()
    set(${_list} ${${_list}} ${${_release}})
  endif()
endmacro()

find_path(BULLET_EXTRAS_INCLUDE_DIR NAMES btBulletCollisionCommon.h
  HINTS
    ${BULLET_ROOT}/include
    ${BULLET_ROOT}/src
  PATH_SUFFIXES bullet
)

# Find the libraries

_FIND_BULLET_LIBRARY(BULLET_FILELOADER_LIBRARY        Bullet2FileLoader)
_FIND_BULLET_LIBRARY(BULLET_FILELOADER_LIBRARY_DEBUG  Bullet2FileLoader_Debug Bullet2FileLoader_d)
_FIND_BULLET_LIBRARY(BULLET_WORLDIMPORTER_LIBRARY        BulletWorldImporter)
_FIND_BULLET_LIBRARY(BULLET_WORLDIMPORTER_LIBRARY_DEBUG  BulletWorldImporter_Debug BulletWorldImporter_d)



#include(${CMAKE_CURRENT_LIST_DIR}/FindPackageHandleStandardArgs.cmake)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(BulletExtras DEFAULT_MSG
    BULLET_FILELOADER_LIBRARY BULLET_WORLDIMPORTER_LIBRARY   BULLET_EXTRAS_INCLUDE_DIR)

set(BULLET_EXTRAS_INCLUDE_DIRS ${BULLET_EXTRAS_INCLUDE_DIR})
if(BulletExtras_FOUND)
   _BULLET_APPEND_LIBRARIES(BULLETEXTRAS_LIBRARIES BULLET_FILELOADER_LIBRARY)
   _BULLET_APPEND_LIBRARIES(BULLETEXTRAS_LIBRARIES BULLET_WORLDIMPORTER_LIBRARY)
endif()
