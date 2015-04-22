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
# Add libpose library
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/libpose" )

IF ( GOOGLE_TEST AND GOOGLE_TEST_PATH )
  INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/${GOOGLE_TEST_PATH}/include" )
ENDIF()

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
