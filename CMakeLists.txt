# # # # # #  project setup  # # # # # #
cmake_minimum_required( VERSION 3.5 )
project( vsgBullet )

set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}" )
INCLUDE( CMakeMacros )

#include( FindVSGHelper )
include( FindBulletHelper )
# Please keep in sync with Version.h
set( VSGBULLET_VERSION 3.00.00 )
set( VSGBCOLLISION_MAJOR_VERSION 3 )
set( VSGBCOLLISION_MINOR_VERSION 0 )
set( VSGBCOLLISION_SUB_VERSION 0 )

find_package(vsg 1.1.10)

vsg_setup_dir_vars()
vsg_setup_build_vars()

# find the optional vsgXchange that can be used for reading a range of image and 3d model formats and shader compilation
find_package(vsgXchange 1.1.6 QUIET)

# find the optional vsgImGui that can be used for GUI elements added into graphics windows.
find_package(vsgImGui QUIET)

SET(CMAKE_DEBUG_POSTFIX "d")

# Setup the parrallel build support for windows
#
IF( WIN32 AND MSVC )
    OPTION( VSGBULLET_WIN32_USE_MP "Build with multiple processes (/MP option; VS 2005 and above)." ON )
    MARK_AS_ADVANCED( VSGBULLET_WIN32_USE_MP )
    IF( VSGBULLET_WIN32_USE_MP )
        SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP" )
    ENDIF( VSGBULLET_WIN32_USE_MP )
ENDIF( WIN32 AND MSVC )

#
# Setup the the lib directory on a per platform basis
#
if(CMAKE_COMPILER_IS_GNUCXX)
    include(GNUInstallDirs)
endif(CMAKE_COMPILER_IS_GNUCXX)

if(NOT DEFINED CMAKE_INSTALL_LIBDIR)
    set(CMAKE_INSTALL_LIBDIR lib)
endif(NOT DEFINED CMAKE_INSTALL_LIBDIR)


IF(NOT ANDROID)
IF(APPLE)

    #build for iOS
    OPTION(VSG_BUILD_PLATFORM_IPHONE "Enable IPhoneSDK Device support" OFF)
    OPTION(VSG_BUILD_PLATFORM_IPHONE_SIMULATOR "Enable IPhoneSDK Simulator support" OFF)

    IF(VSG_BUILD_PLATFORM_IPHONE OR VSG_BUILD_PLATFORM_IPHONE_SIMULATOR)

        #you need to manually set the default sdk version here
        SET (IPHONE_SDKVER "6.0")

        #the below is taken from ogre, it states the gcc stuff needs to happen before PROJECT() is called. I've no clue if we even need it
        # Force gcc <= 4.2 on iPhone
        include(CMakeForceCompiler)
        CMAKE_FORCE_C_COMPILER(llvm-gcc-4.2 GNU)
        CMAKE_FORCE_CXX_COMPILER(llvm-gcc-4.2 GNU)
        SET(GCC_THUMB_SUPPORT NO)

        #set either the device sdk or the simulator sdk. Can't find away to separate these in the same project
        IF(VSG_BUILD_PLATFORM_IPHONE)
            SET (IPHONE_DEVROOT "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Developer")
            SET (IPHONE_SDKROOT "${IPHONE_DEVROOT}/SDKs/iPhoneOS${IPHONE_SDKVER}.sdk")
        ELSE()
            SET (IPHONE_DEVROOT "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneSimulator.platform/Developer")
            SET (IPHONE_SDKROOT "${IPHONE_DEVROOT}/SDKs/iPhoneSimulator${IPHONE_SDKVER}.sdk")
        ENDIF()
    ENDIF ()

    OPTION(VSG_COMPILE_FRAMEWORKS "compile frameworks instead of dylibs (experimental)" OFF)
    SET(VSG_COMPILE_FRAMEWORKS_INSTALL_NAME_DIR "@executable_path/../Frameworks" CACHE STRING "install name dir for compiled frameworks")
ENDIF()
ENDIF()

IF(APPLE)

        #Here we check if the user specified IPhone SDK
    IF(VSG_BUILD_PLATFORM_IPHONE OR VSG_BUILD_PLATFORM_IPHONE_SIMULATOR)

        #set iphone arch and flags taken from http://sites.google.com/site/michaelsafyan/coding/resources/how-to-guides/cross-compile-for-the-iphone/how-to-cross-compile-for-the-iphone-using-cmake
        IF(VSG_BUILD_PLATFORM_IPHONE)
            SET(CMAKE_OSX_ARCHITECTURES "armv7" CACHE STRING "Build architectures for iOS" FORCE)
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -miphoneos-version-min=5.1 -mno-thumb -arch armv7 -pipe -no-cpp-precomp" CACHE STRING "Flags used by the compiler during all build types." FORCE)
        ELSE()
            #simulator uses i386 architectures
            SET(CMAKE_OSX_ARCHITECTURES "i386" CACHE STRING "Build architectures for iOS Simulator" FORCE)
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mno-thumb -arch i386 -pipe -no-cpp-precomp" CACHE STRING "Flags used by the compiler during all build types." FORCE)
        ENDIF()

        #here we set the specific iphone sdk version. We can only set either device or simulator sdk. So if you want both you currently have to have two seperate projects
        SET(CMAKE_OSX_SYSROOT "${IPHONE_SDKROOT}" CACHE STRING "System root for iOS" FORCE)

        #hack, force link to opengles
        set(CMAKE_EXE_LINKER_FLAGS "-framework Foundation -framework OpenGLES")

        #use the IPhone windowing system
        SET(VSG_WINDOWING_SYSTEM "IOS" CACHE STRING "Forced IPhone windowing system on iOS"  FORCE)
        SET(VSG_DEFAULT_IMAGE_PLUGIN_FOR_OSX "imageio" CACHE STRING "Forced imageio default image plugin for iOS" FORCE)

        #I think this or similar will be required for IPhone apps
        OPTION(VSG_BUILD_APPLICATION_BUNDLES "Enable the building of applications and examples as OSX Bundles" ON)
    ENDIF()
ENDIF()


# VSGBULLET_USE_DOUBLE_PRECISION gets set to TRUE or FALSE in FindBulletHelper.cmake.
# If TRUE, add the BT double precision definition globally.
if( VSGBULLET_USE_DOUBLE_PRECISION )
    add_definitions( -DBT_USE_DOUBLE_PRECISION )
endif()


option( VSGBULLET_P5_SUPPORT "Enable to include support for the P5 glove and build the related test code" OFF )
if( VSGBULLET_P5_SUPPORT )
    add_definitions( -DUSE_P5 )
    set( VSGBULLET_P5_INCLUDE_DIR "" CACHE PATH "" )
    set( VSGBULLET_P5_LIBRARY "" CACHE FILEPATH "" )
else()
    unset( VSGBULLET_P5_INCLUDE_DIR CACHE )
    unset( VSGBULLET_P5_LIBRARY CACHE )
endif()


INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${VSG_INCLUDE_DIRS}
    #${VSGWORKS_INCLUDE_DIR}
    ${BULLET_INCLUDE_DIRS}
    ${BULLET_EXTRAS_INCLUDE_DIR}
    ${VSGBULLET_P5_INCLUDE_DIR}
)


# Support for collision-only (no dynamics)
if( BULLET_DYNAMICS_FOUND )
    add_definitions( -DUSE_BULLET_DYNAMICS )
endif()

option( BUILD_SHARED_LIBS "Set to ON to build vsgBullet for dynamic linking. Use OFF for static." ON )
if( BUILD_SHARED_LIBS )
    add_definitions( -DVSGBULLET_SHARED )
else()
    add_definitions( -DVSGBULLET_STATIC )
endif()



OPTION( VSGBULLET_BUILD_APPLICATIONS "Enable to build applications" ON )
IF( VSGBULLET_BUILD_APPLICATIONS )
    ADD_SUBDIRECTORY( applications )
ENDIF( VSGBULLET_BUILD_APPLICATIONS )

OPTION( VSGBULLET_BUILD_EXAMPLES "Enable to build example code" ON )
IF( VSGBULLET_BUILD_EXAMPLES )
    ADD_SUBDIRECTORY( examples )
ENDIF( VSGBULLET_BUILD_EXAMPLES )

option( VSGBULLET_BUILD_TESTS "Enable to build test code" OFF )
if( VSGBULLET_BUILD_TESTS )
    enable_testing()
    add_subdirectory( tests )
endif()

option( VSGBULLET_INSTALL_DATA "Enable to add the data directory to the install target" ON )
if( VSGBULLET_INSTALL_DATA )
    add_subdirectory( data )
endif()


set( VSGBULLET_VERSION 3.00.00 )
ADD_SUBDIRECTORY( src/vsgbCollision )
ADD_SUBDIRECTORY( src/vsgbDynamics )
ADD_SUBDIRECTORY( src/vsgbInteraction )
#ADD_SUBDIRECTORY( src/plugins )
#ADD_SUBDIRECTORY( vsgbulletserializer )
#
# Doxygen
find_package( Doxygen )
if( DOXYGEN_FOUND )
    set( HAVE_DOT "NO" )
    if( DOXYGEN_DOT_PATH )
        set( HAVE_DOT "YES" )
    endif()

    configure_file( ${PROJECT_SOURCE_DIR}/doc/doxyfile.cmake
        ${PROJECT_BINARY_DIR}/doc/doxyfile
    )
    add_custom_target( Documentation ${DOXYGEN_EXECUTABLE}
        ${PROJECT_BINARY_DIR}/doc/doxyfile
    )
endif()

# On windows provide the user with the pdb files
# for debugging if they are present
if(MSVC)
    install(
        DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/\${CMAKE_INSTALL_CONFIG_NAME}/
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
        USE_SOURCE_PERMISSIONS
        COMPONENT libvsgbbullet-dev
        FILES_MATCHING PATTERN "*.pdb"
    )
endif(MSVC)

#
# Create and install all of the auto find tools
# =============================================

# Add all targets to the build-tree export set
export(TARGETS vsgbInteraction vsgbCollision vsgbDynamics FILE "${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/vsgBulletLibraryDepends.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE vsgBullet)

# Create a vsgBulletBuildTreeSettings.cmake file for the use from the build tree
configure_file(vsgBulletBuildTreeSettings.cmake.in
  "${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/vsgBulletBuildTreeSettings.cmake" @ONLY)

#configure_file(vsgBulletConfig.cmake.in
#  "${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/vsgBulletConfig.cmake" @ONLY)
configure_file(vsgBulletConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/vsgBulletConfigVersion.cmake" @ONLY)
configure_file(UsevsgBullet.cmake.in
  "${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/UsevsgBullet.cmake" @ONLY)

# Install the export set for use with the install-tree
install(EXPORT vsgbullet-targets DESTINATION ${CMAKE_INSTALL_LIBDIR})

include( CMakePackageConfigHelpers )
# Create the vsgBulletConfig.cmake and vsgBulletConfigVersion files
#file(RELATIVE_PATH CONF_REL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
#   "${CMAKE_INSTALL_PREFIX}/include")

configure_package_config_file(
    vsgBulletConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/vsgBulletConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PATH_VARS VSGBULLET_USE_DOUBLE_PRECISION )

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/vsgBulletConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/vsgBulletConfigVersion.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/UsevsgBullet.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR} )

#
# Setup the fpc auto detection files
if(WIN32)
    set(VSGBULLET_FPC_LIBRARIES "/libpath:\"\${libdir}\" vsgbCollision.lib vsgbDynamics.lib vsgbInteraction.lib")
    set(VSGBULLET_FPC_INCLUDES "/I\"\${includedir}\"")
else(WIN32)
    set(VSGBULLET_FPC_INCLUDES "-I\${includedir}")
    if(APPLE)
        set(VSGBULLET_FPC_LIBRARIES "-L\${libdir} -lvsgbCollision -lvsgbDynamics -lvsgbInteraction")
    else(APPLE)
        set(VSGBULLET_FPC_LIBRARIES "-L\${libdir} -lvsgbCollision -lvsgbDynamics -lvsgbInteraction")
    endif(APPLE)
endif(WIN32)

configure_file(extra/vsgbullet.fpc.in
  "${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/flagpoll/vsgbullet.fpc" @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/flagpoll/vsgbullet.fpc
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/flagpoll )


vsg_add_feature_summary()
