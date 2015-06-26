# Tests

# Configure modules path
SET ( MODULE_PATH "${${PROJECT_NAME}_SOURCE_DIR}/cmake" )
INCLUDE ( "${MODULE_PATH}/modules.cmake" )

INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}" )
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/${GOOGLE_TEST_PATH}" )
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/${GOOGLE_TEST_PATH}/include" )
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/${LIBPOSE_MODULE}" )
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/utils/detectorTests" )
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/utils/general" )
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/utils/solverTests" )
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/utils/parameterTuners" )

INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/${TINYXML2_PATH}" )

OPTION ( DISABLE_PTHREADS "Do not use pthreads" OFF )

# Module dependencies
# Ensure that those targets are built before current one
SET ( ${TESTS_MODULE}_DEPENDS "GoogleTestFramework" ${LIBPOSE_MODULE} )

IF ( NOT DISABLE_PTHREADS )
# Defines CMAKE_USE_PTHREADS_INIT and CMAKE_THREAD_LIBS_INIT.
  FIND_PACKAGE ( Threads )
endif()

IF ( CMAKE_USE_PTHREADS_INIT )  # The pthreads library is available and allowed.
  ADD_DEFINITIONS ( -DGTEST_HAS_PTHREAD=1 )
  LIST ( APPEND ${TESTS_MODULE}_LNK ${CMAKE_THREAD_LIBS_INIT} )
else()
  ADD_DEFINITIONS ( -DGTEST_HAS_PTHREAD=0 )
endif()

# main test file
LIST ( APPEND ${TESTS_MODULE}_SRC ${${PROJECT_NAME}_SOURCE_DIR}/${GOOGLE_TEST_PATH}/src/gtest-all test_exec )

# project loader: need for some tests with input data
LIST ( APPEND ${TESTS_MODULE}_SRC ${${PROJECT_NAME}_SOURCE_DIR}/utils/general/projectLoader )
LIST ( APPEND ${TESTS_MODULE}_SRC ${${PROJECT_NAME}_SOURCE_DIR}/${TINYXML2_PATH}/tinyxml2.cpp )

# libpose tests
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/bodyJoint_get_and_set_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/bodyJoint_constructor_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/bodyJoint_operators_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/bodyPart_get_and_set_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/bodyPart_constructor_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/bodyPart_operators_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/keyframe_get_and_set_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/lockframe_get_and_set_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/interpolation_get_and_set_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/frame_general_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/skeleton_get_and_set_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/skeleton_operators_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/colorHistDetector_get_and_set_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/colorHistDetector_private_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/colorHistDetector_train_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/colorHistDetector_operators_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/hogDetector_get_and_set_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/surfDetector_get_and_set_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/score_get_and_set_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/score_constructor_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/score_operators_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/poseHelper_poserect_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/poseHelper_rotatepoint_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/poseHelper_angle_dist_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/skeleton_infer_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/detector_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/limbLabel_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/nskpsolver_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/frames_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/hogdetector_tests )
LIST ( APPEND ${TESTS_MODULE}_SRC libpose/surfdetector_tests )

LIST ( APPEND ${TESTS_MODULE}_LNK ${LIBPOSE_MODULE} )
LIST ( APPEND ${TESTS_MODULE}_LNK ${OpenCV_LIBS} )
LIST ( APPEND ${TESTS_MODULE}_LNK ${HDF5_LIBRARIES} )


ADD_EXECUTABLE ( ${TESTS_MODULE} ${${TESTS_MODULE}_SRC} )

# TODO: Library names should be taken from variables
TARGET_LINK_LIBRARIES ( ${TESTS_MODULE} ${${TESTS_MODULE}_LNK} )

ADD_CUSTOM_COMMAND ( TARGET ${TESTS_MODULE} PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${${PROJECT_NAME}_SOURCE_DIR}/tests/testdata1 $<TARGET_FILE_DIR:${TESTS_MODULE}>/${TESTS_MODULE}_TestData/testdata1 )
ADD_CUSTOM_COMMAND ( TARGET ${TESTS_MODULE} PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${${PROJECT_NAME}_SOURCE_DIR}/tests/ImageRotationTestData $<TARGET_FILE_DIR:${TESTS_MODULE}>/${TESTS_MODULE}_TestData/ImageRotationTestData )
ADD_CUSTOM_COMMAND ( TARGET ${TESTS_MODULE} PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${${PROJECT_NAME}_SOURCE_DIR}/tests/CHDTrainTestData $<TARGET_FILE_DIR:${TESTS_MODULE}>/${TESTS_MODULE}_TestData/CHDTrainTestData )
ADD_CUSTOM_COMMAND ( TARGET ${TESTS_MODULE} PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${${PROJECT_NAME}_SOURCE_DIR}/tests/CHDTrainTestData $<TARGET_FILE_DIR:${TESTS_MODULE}>/${TESTS_MODULE}_TestData/SurfDetectorTestsData )

# Installation instructions for the test module
# Please specify the installation instructions sprcific
# To the architecture the project is being built for.

IF ( UNIX )
    INSTALL (
            TARGETS ${TESTS_MODULE}
            RUNTIME DESTINATION bin/${ROOT_PROJECT_NAME}
            LIBRARY DESTINATION lib/${ROOT_PROJECT_NAME}
            ARCHIVE DESTINATION lib/static/${ROOT_PROJECT_NAME}
    )
ELSEIF ( WIN32 )
    INSTALL (
            TARGETS ${TESTS_MODULE}
            RUNTIME DESTINATION bin/
            LIBRARY DESTINATION lib/
            ARCHIVE DESTINATION lib/static/
    )
ENDIF()

