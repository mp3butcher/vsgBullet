set( LIB_NAME vsgbInteraction )
set( HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME} )
add_definitions( -DVSGBINTERACTION_LIBRARY )

if( VSGBULLET_P5_SUPPORT )
    set( P5_HEADER ${HEADER_PATH}/p5support.h )
    set( P5_SOURCE p5support.cpp )
else()
    unset( P5_HEADER )
    unset( P5_SOURCE )
endif()


set( LIB_PUBLIC_HEADERS
    ${P5_HEADER}
    ${HEADER_PATH}/Export.h
    ${HEADER_PATH}/ArticulationRecord.h
    ${HEADER_PATH}/DragHandler.h
    ${HEADER_PATH}/LaunchHandler.h
    ${HEADER_PATH}/GestureHandler.h
    ${HEADER_PATH}/HandNode.h
    ${HEADER_PATH}/HandTestEventHandler.h
    ${HEADER_PATH}/SaveRestoreHandler.h
)

set( _vsgBulletSourceFiles
    ${P5_SOURCE}
    ArticulationRecord.cpp
    DragHandler.cpp
    LaunchHandler.cpp
   # GestureHandler.cpp
   # HandNode.cpp
   # HandTestEventHandler.cpp
    SaveRestoreHandler.cpp
)

add_library( ${LIB_NAME}
    ${LIB_PUBLIC_HEADERS}
    ${_vsgBulletSourceFiles}
)
if( BUILD_SHARED_LIBS )
    target_link_libraries( ${LIB_NAME}
        vsg::vsg
        ${BULLET_LIBRARIES}
        ${VSGBULLET_P5_LIBRARY}
    )

    # vsgbDynamics depends on vsgbCollision
    target_link_libraries( ${LIB_NAME}
        optimized "vsgbDynamics"
        debug "vsgbDynamics"
        optimized "vsgbCollision"
        debug "vsgbCollision"
        ${VSGBULLET_P5_LIBRARY}
    )
endif()

set_target_properties( ${LIB_NAME} PROPERTIES VERSION ${VSGBULLET_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES SOVERSION ${VSGBULLET_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES PROJECT_LABEL "Lib ${LIB_NAME}" )

include( ModuleInstall REQUIRED )
