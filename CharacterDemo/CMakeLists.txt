# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  


# You shouldn't have to modify anything below this line 
########################################################


INCLUDE_DIRECTORIES(
 ${TUXGAL_SOURCE_DIR}/GlutBtApplication
 ${TUXGAL_SOURCE_DIR}/BspLoader
)

LINK_LIBRARIES(
 GlutBtApplication BspLoader BulletDynamics BulletCollision LinearMath    ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)
SET(CharacterDemo_SRCS

	DynamicCharacterController.cpp
	DynamicCharacterController.h
	CharacterDemo.cpp
	CharacterDemo.h
	main.cpp
)

	ADD_EXECUTABLE(CharacterDemo
		${CharacterDemo_SRCS}
	)

IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
	ADD_CUSTOM_COMMAND(
		TARGET CharacterDemo
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${TUXGAL_SOURCE_DIR}/data/BspDemo.bsp ${CMAKE_CURRENT_BINARY_DIR}
	)
ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)



IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(CharacterDemo PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(CharacterDemo PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(CharacterDemo PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
