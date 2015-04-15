# libposegui module
# Creates a libpose graphic user interface 
cmake_minimum_required(VERSION 2.8.11)

#project(LibposeGUI)

#set clang
    #SET (CMAKE_C_COMPILER             "/usr/bin/clang")
    #SET (CMAKE_C_FLAGS                "-Wall -std=c++11")
    #SET (CMAKE_C_FLAGS_DEBUG          "-g")
    #SET (CMAKE_C_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
    #SET (CMAKE_C_FLAGS_RELEASE        "-O4 -DNDEBUG")
    #SET (CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g")

    #SET (CMAKE_CXX_COMPILER             "/usr/bin/clang++")
    #SET (CMAKE_CXX_FLAGS                "-Wall -std=c++11")
    #SET (CMAKE_CXX_FLAGS_DEBUG          "-g")
    #SET (CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
    #SET (CMAKE_CXX_FLAGS_RELEASE        "-O4 -DNDEBUG")
    #SET (CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")

    #SET (CMAKE_AR      "/usr/bin/llvm-ar")
    #SET (CMAKE_LINKER  "/usr/bin/llvm-ld")
    #SET (CMAKE_NM      "/usr/bin/llvm-nm")
    #SET (CMAKE_OBJDUMP "/usr/bin/llvm-objdump")
    #SET (CMAKE_RANLIB  "/usr/bin/llvm-ranlib")


# Configure modules path
SET ( MODULE_PATH "${${PROJECT_NAME}_SOURCE_DIR}/cmake" )
INCLUDE ( "${MODULE_PATH}/modules.cmake" )
# Add thirdparty software
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/../thirdparty/tree/src" )
# Add libpose library
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/libpose" )
#Add OpenCV library
FIND_PACKAGE ( OpenCV REQUIRED )
INCLUDE_DIRECTORIES ( ${OpenCV_INCLUDE_DIRS} )
# use OpenGM
FIND_PACKAGE ( OpenGM REQUIRED )
INCLUDE_DIRECTORIES ( ${OpenGM_INCLUDE_DIR} )
# use HDF5
IF ( WIN32 )
# FindHDF5 is broken on Windows
# it wrongly discovers the dlls instead of the lib files
# see: www.cmake.org/Bug/bug_relationship_graph.php?bug_id=14111
# therefore we set the variable manually
  FIND_LIBRARY ( HDF5_IMPORT_LIB NAMES hdf5 hdf5dll )
  FIND_LIBRARY ( HDF5_HL_IMPORT_LIB NAMES hdf5_hl hdf5_hldll )
  FIND_PATH ( HDF5_INCLUDE_DIR hdf5.h )
  SET ( HDF5_LIBRARIES ${HDF5_IMPORT_LIB} ${HDF5_HL_IMPORT_LIB} )
  MESSAGE ( STATUS "HDF5: autodiscovery is broken on WIN32; using global search paths")
  MESSAGE ( STATUS "HDF5: found ${HDF5_LIBRARIES}")
ELSE ()
  FIND_PACKAGE ( HDF5 REQUIRED )
ENDIF ()
INCLUDE_DIRECTORIES ( ${HDF5_INCLUDE_DIR} )
ADD_DEFINITIONS ( ${HDF5_CPPFLAGS} )

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

#Find the QtCore library
find_package(Qt5Core)
# Find the QtWidgets library
find_package(Qt5Widgets)
# Find the QtXml library
find_package(Qt5Xml)
#Find the QtXmlPatterns library
find_package(Qt5XmlPatterns)
#Find the OpenGL library
find_package(Qt5OpenGL)

#Add sources files

#Main files
LIST(APPEND ${LIBPOSEGUI_MODULE}_SRC main.cpp)
LIST(APPEND ${LIBPOSEGUI_MODULE}_SRC mainwindow.cpp)
#Custom layouts
LIST(APPEND ${LIBPOSEGUI_MODULE}_SRC flowlayout.cpp)
#Custom group boxes
LIST(APPEND ${LIBPOSEGUI_MODULE}_SRC frameboxwidget.cpp)
LIST(APPEND ${LIBPOSEGUI_MODULE}_SRC solveboxwidget.cpp)
LIST(APPEND ${LIBPOSEGUI_MODULE}_SRC toolboxwidget.cpp)
#Custom table widget
LIST(APPEND ${LIBPOSEGUI_MODULE}_SRC frametablewidget.cpp)
#View helper
LIST(APPEND ${LIBPOSEGUI_MODULE}_SRC frameview2d.cpp)
LIST(APPEND ${LIBPOSEGUI_MODULE}_SRC frameview3d.cpp)
#Project file
LIST(APPEND ${LIBPOSEGUI_MODULE}_SRC project.cpp)
#Utility helper class
LIST(APPEND ${LIBPOSEGUI_MODULE}_SRC utility.cpp)
#Custom graphick items
LIST(APPEND ${LIBPOSEGUI_MODULE}_SRC bodyjointitem.cpp)
LIST(APPEND ${LIBPOSEGUI_MODULE}_SRC bodypartitem.cpp)
#Xml message handler
LIST(APPEND ${LIBPOSEGUI_MODULE}_SRC xmlmessagehandler.cpp)
#Attributes of xml project
LIST(APPEND ${LIBPOSEGUI_MODULE}_SRC projectattr.h)

#Add forms
QT5_WRAP_UI(UIS_HDRS
    mainwindow.ui
)

#Add resources files
QT5_ADD_RESOURCES( ${LIBPOSEGUI_MODULE}_RRC resources.qrc )


# Tell CMake to create the executable
add_executable(${LIBPOSEGUI_MODULE} ${${LIBPOSEGUI_MODULE}_SRC} ${UIS_HDRS} ${${LIBPOSEGUI_MODULE}_RRC} )

if (WIN32)
  set (QtMain Qt5::WinMain)
endif()

# Add libraries
target_link_libraries( ${LIBPOSEGUI_MODULE}
    ${QtMain}
    Qt5::Core
    Qt5::Widgets
    Qt5::Xml
    Qt5::XmlPatterns
    Qt5::OpenGL
    ${LIBPOSE_MODULE}
    ${OpenCV_LIBS}
    ${HDF5_LIBRARIES}
)

# Hide console window on Windows system
if (WIN32)
  if (MSVC)
    set_target_properties(${LIBPOSEGUI_MODULE} PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
  elseif(CMAKE_COMPILER_IS_GNUCXX)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mwindows")
  endif()
endif()
