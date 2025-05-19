if( WIN32 )
    set( CMAKE_DEBUG_POSTFIX d )
endif()

macro( link_internal TRGTNAME )
    foreach( LINKLIB ${ARGN} )
        target_link_libraries( ${TRGTNAME} optimized "${LINKLIB}" debug "${LINKLIB}" )
    endforeach()
endmacro()


macro( _vsgBulletPlugin TRGTNAME )
    if( BUILD_SHARED_LIBS )
        add_library( ${TRGTNAME} MODULE ${ARGN} )

        # TBD. This is a shortcut. It means any app that wants to be
        # collision-only, for example, and also use the dot VSG support,
        # must link with all three libs. FIXME.
        link_internal( ${TRGTNAME}
            vsgbInteraction
            vsgbDynamics
            vsgbCollision
        )
        target_link_libraries( ${TRGTNAME}
            ${VSGWORKS_LIBRARIES}
              vsg::vsg
        )
    else()
        add_library( ${TRGTNAME} STATIC ${ARGN} )
    endif()

    if( WIN32 )
        set_target_properties( ${TRGTNAME} PROPERTIES DEBUG_POSTFIX d )
    endif()
    set_target_properties( ${TRGTNAME} PROPERTIES PREFIX "" )
    set_target_properties( ${TRGTNAME} PROPERTIES PROJECT_LABEL "Plugin ${TRGTNAME}" )
endmacro()


macro( _vsgBulletMakeInteractionExe _exeName )
    set( _vsgBulletLibs
        "vsgbInteraction;vsgbDynamics;vsgbCollision"
    )
    set( _bulletLibs
        "${BULLET_LIBRARIES}"
        "${VSGBULLET_P5_LIBRARY}"
        vsg::vsg
    )
    _vsgBulletMakeExeInternal( ${_exeName} "${_vsgBulletLibs}" "${_bulletLibs}" ${ARGN} )
endmacro()

macro( _vsgBulletMakeDynamicsExe _exeName )
    set( _vsgBulletLibs
        "vsgbDynamics;vsgbCollision"
    )
    set( _bulletLibs
        "${BULLET_LIBRARIES}"
         vsg::vsg
    )
    _vsgBulletMakeExeInternal( ${_exeName} "${_vsgBulletLibs}" "${_bulletLibs}" ${ARGN} )
endmacro()

macro( _vsgBulletMakeCollisionExe _exeName )
    set( _vsgBulletLibs
        "vsgbCollision"
    )
    if( BULLET_COLLISION_LIBRARY_DEBUG AND BULLET_MATH_LIBRARY_DEBUG )
        set( _bulletLibs
            optimized ${BULLET_COLLISION_LIBRARY} debug ${BULLET_COLLISION_LIBRARY_DEBUG}
            optimized ${BULLET_MATH_LIBRARY} debug ${BULLET_MATH_LIBRARY_DEBUG}
        )
    else()
        set( _bulletLibs
            ${BULLET_COLLISION_LIBRARY}
            ${BULLET_MATH_LIBRARY}
            vsg::vsg
        )
    endif()
    _vsgBulletMakeExeInternal( ${_exeName} "${_vsgBulletLibs}" "${_bulletLibs}" ${ARGN} )
endmacro()

macro( _vsgBulletMakeExeInternal _exeName _vsgBulletLibs _bulletLibs )
    add_executable( ${_exeName} ${ARGN} )
    if( WIN32 )
        set_target_properties( ${_exeName} PROPERTIES DEBUG_POSTFIX d )
    endif()

    link_internal( ${_exeName}
        ${_vsgBulletLibs}
    )
    target_link_libraries( ${_exeName}
        ${VSGWORKS_LIBRARIES}
        ${_bulletLibs}
        vsg::vsg
     )
    if( ${CATEGORY} STREQUAL "App" )
        install(
            TARGETS ${_exeName}
            RUNTIME DESTINATION bin COMPONENT vsgbullet
        )
    else()
        install(
            TARGETS ${_exeName}
            RUNTIME DESTINATION share/${CMAKE_PROJECT_NAME}/bin COMPONENT vsgbullet
        )
    endif()
    set_target_properties( ${_exeName} PROPERTIES PROJECT_LABEL "${CATEGORY} ${_exeName}" )
endmacro()
