# spel module
# Creates a spel static library 

# spel module parameters

# Configure modules path
SET ( MODULE_PATH "${${PROJECT_NAME}_SOURCE_DIR}/cmake" )
INCLUDE ( "${MODULE_PATH}/modules.cmake" )

INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}" )
IF ( GOOGLE_TEST AND GOOGLE_TEST_PATH )
  INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/${GOOGLE_TEST_PATH}/include" )
ENDIF()

# Command line options, their defaults and checks

# Set correct defines and include correct files
LIST ( APPEND ${SPEL_MODULE}_SRC skeleton)
LIST ( APPEND ${SPEL_MODULE}_SRC limbLabel)
LIST ( APPEND ${SPEL_MODULE}_SRC frame)
LIST ( APPEND ${SPEL_MODULE}_SRC colorHistDetector)
LIST ( APPEND ${SPEL_MODULE}_SRC surfDetector)
LIST ( APPEND ${SPEL_MODULE}_SRC hogDetector)
LIST ( APPEND ${SPEL_MODULE}_SRC score)
LIST ( APPEND ${SPEL_MODULE}_SRC bodyJoint)
LIST ( APPEND ${SPEL_MODULE}_SRC bodyPart)
LIST ( APPEND ${SPEL_MODULE}_SRC keyframe)
LIST ( APPEND ${SPEL_MODULE}_SRC interpolation)
LIST ( APPEND ${SPEL_MODULE}_SRC lockframe)
LIST ( APPEND ${SPEL_MODULE}_SRC solvlet)
LIST ( APPEND ${SPEL_MODULE}_SRC solver)
LIST ( APPEND ${SPEL_MODULE}_SRC poseHelper)
LIST ( APPEND ${SPEL_MODULE}_SRC imagesimilaritymatrix)
LIST ( APPEND ${SPEL_MODULE}_SRC minspanningtree)
LIST ( APPEND ${SPEL_MODULE}_SRC nskpsolver)
LIST ( APPEND ${SPEL_MODULE}_SRC tlpssolver)
LIST ( APPEND ${SPEL_MODULE}_SRC detector)
LIST ( APPEND ${SPEL_MODULE}_SRC sequence)


# Create a static library
ADD_LIBRARY ( ${SPEL_MODULE} STATIC ${${SPEL_MODULE}_SRC})

# Installation instructions for the spel module
# Please specify the installation instructions sprcific 
# To the architecture the project is being built for.

IF ( UNIX )
	INSTALL (
		TARGETS ${SPEL_MODULE}
		RUNTIME DESTINATION bin/${ROOT_PROJECT_NAME}
		LIBRARY DESTINATION lib/${ROOT_PROJECT_NAME}
		ARCHIVE DESTINATION lib/static/${ROOT_PROJECT_NAME}
	)
ELSEIF ( WIN32 )
	INSTALL (
		TARGETS ${SPEL_MODULE}
		RUNTIME DESTINATION bin/
		LIBRARY DESTINATION lib/
		ARCHIVE DESTINATION lib/static/
	)
ENDIF()

