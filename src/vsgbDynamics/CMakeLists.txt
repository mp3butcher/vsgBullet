set( LIB_NAME vsgbDynamics )
set( HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME} )
add_definitions( -DVSGBDYNAMICS_LIBRARY )

INCLUDE_DIRECTORIES(${BULLET_INCLUDE_DIR}/../Extras/ConvexDecomposition)

set( LIB_PUBLIC_HEADERS
    ${HEADER_PATH}/Export.h
    ${HEADER_PATH}/Constraints.h
    ${HEADER_PATH}/CreationRecord.h
    ${HEADER_PATH}/GroundPlane.h
    ${HEADER_PATH}/MotionState.h
    ${HEADER_PATH}/PhysicsState.h
    ${HEADER_PATH}/PhysicsThread.h
    ${HEADER_PATH}/RigidBody.h
    ${HEADER_PATH}/Joint.h
    ${HEADER_PATH}/SoftBody.h
    ${HEADER_PATH}/RigidBodyAnimation.h
    ${HEADER_PATH}/TripleBuffer.h
    ${HEADER_PATH}/World.h

)

set( _vsgBulletSourceFiles
    Constraints.cpp
    CreationRecord.cpp
    GroundPlane.cpp
    MotionState.cpp
    PhysicsState.cpp
    PhysicsThread.cpp
    RigidBody.cpp
    SoftBody.cpp
    Joint.cpp
    RigidBodyAnimation.cpp
    TripleBuffer.cpp
    World.cpp
)

add_library( ${LIB_NAME}
    ${LIB_PUBLIC_HEADERS}
    ${_vsgBulletSourceFiles}
)
if( BUILD_SHARED_LIBS )
    target_link_libraries( ${LIB_NAME}
        vsg::vsg  ${VSGANIMATION_LIBRARY} vsgbCollision
        ${BULLET_LIBRARIES} ${BULLETEXTRAS_LIBRARIES} ${BULLET_EXTRA_CONVEXDECOMPOSITION_LIBRARY}
    )

    # vsgbDynamics depends on vsgbCollision
  #  target_link_libraries( ${LIB_NAME} vsgbCollision
  #         ${BULLET_LIBRARIES}
   # )
endif()

set_target_properties( ${LIB_NAME} PROPERTIES VERSION ${VSGBULLET_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES SOVERSION ${VSGBULLET_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES PROJECT_LABEL "Lib ${LIB_NAME}" )

include( ModuleInstall REQUIRED )
