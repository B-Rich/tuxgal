# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  


# You shouldn't have to modify anything below this line 
########################################################


LINK_LIBRARIES(
    tux
    boost_system ${Boost_LIBRARIES}
    ${BULLET_LIBRARIES}
    ${OGRE_LIBRARIES}
    ${OIS_LIBRARIES}
    )

SET(tuxgal_SRCS
    tuxInputManager.cpp
    tuxOgreApplication.cpp
    main.cpp
    )

ADD_EXECUTABLE(tuxgal
    ${tuxgal_SRCS}
    )

IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
    ADD_CUSTOM_COMMAND(
        TARGET tuxgal
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${TUXGAL_SOURCE_DIR}/data/cube.mesh ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${TUXGAL_SOURCE_DIR}/data/Ice.material ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${TUXGAL_SOURCE_DIR}/data/ice.jpg ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${TUXGAL_SOURCE_DIR}/data/sphere.mesh ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${TUXGAL_SOURCE_DIR}/data/Snow.material ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${TUXGAL_SOURCE_DIR}/data/snow.jpg ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${TUXGAL_SOURCE_DIR}/data/penguin.mesh ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${TUXGAL_SOURCE_DIR}/data/penguin.skeleton ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${TUXGAL_SOURCE_DIR}/data/Penguin.material ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${TUXGAL_SOURCE_DIR}/data/penguin.jpg ${CMAKE_CURRENT_BINARY_DIR}
    )
ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
    SET_TARGET_PROPERTIES(tuxgal PROPERTIES  DEBUG_POSTFIX "_Debug")
    SET_TARGET_PROPERTIES(tuxgal PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
    SET_TARGET_PROPERTIES(tuxgal PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)

