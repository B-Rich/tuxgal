# This is basically the overall name of the project in Visual Studio this is the name of the Solution File


# For every executable you have with a main method you should have an add_executable line below.
# For every add executable line you should list every .cpp and .h file you have associated with that executable.  


# You shouldn't have to modify anything below this line 
########################################################


INCLUDE_DIRECTORIES(
 ${TUXGAL_SOURCE_DIR}/GlutBtApplication
)

LINK_LIBRARIES(
 tux GlutBtApplication ${BULLET_LIBRARIES} ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)
SET(tuxgal_SRCS
        tuxGlutBtApplication.cpp
	main.cpp
)

	ADD_EXECUTABLE(tuxgal
		${tuxgal_SRCS}
	)



IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(tuxgal PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(tuxgal PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(tuxgal PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
