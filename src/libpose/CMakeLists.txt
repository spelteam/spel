# libpose module
# Creates a libpose static library 

# libpose module parameters

# Configure modules path
SET ( MODULE_PATH "${${PROJECT_NAME}_SOURCE_DIR}/cmake" )
INCLUDE ( "${MODULE_PATH}/modules.cmake" )

INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}" )
IF ( GOOGLE_TEST AND GOOGLE_TEST_PATH )
  INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/${GOOGLE_TEST_PATH}/include" )
ENDIF()
FIND_PACKAGE ( OpenCV REQUIRED )
INCLUDE_DIRECTORIES ( ${OpenCV_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/../thirdparty/tree/src" )

# Command line options, their defaults and checks

# Set correct defines and include correct files
LIST ( APPEND ${LIBPOSE_MODULE}_SRC skeleton)
LIST ( APPEND ${LIBPOSE_MODULE}_SRC limbLabel)
LIST ( APPEND ${LIBPOSE_MODULE}_SRC frame)
LIST ( APPEND ${LIBPOSE_MODULE}_SRC colorHistDetector)
LIST ( APPEND ${LIBPOSE_MODULE}_SRC surfDetector)
LIST ( APPEND ${LIBPOSE_MODULE}_SRC hogDetector)
LIST ( APPEND ${LIBPOSE_MODULE}_SRC score)


# Create a static library
ADD_LIBRARY ( ${LIBPOSE_MODULE} STATIC ${${LIBPOSE_MODULE}_SRC})

# Installation instructions for the libpose module
# Please specify the installation instructions sprcific 
# To the architecture the project is being built for.

IF ( UNIX )
	INSTALL (
		TARGETS ${LIBPOSE_MODULE}
		RUNTIME DESTINATION bin/${ROOT_PROJECT_NAME}
		LIBRARY DESTINATION lib/${ROOT_PROJECT_NAME}
		ARCHIVE DESTINATION lib/static/${ROOT_PROJECT_NAME}
	)
ELSEIF ( WIN32 )
	INSTALL (
		TARGETS ${LIBPOSE_MODULE}
		RUNTIME DESTINATION bin/
		LIBRARY DESTINATION lib/
		ARCHIVE DESTINATION lib/static/
	)
ENDIF()

