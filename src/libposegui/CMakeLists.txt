# libposegui module
# Creates a libpose graphic user interface 
cmake_minimum_required(VERSION 2.8.11)

#project(LibposeGUI)

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
    ${LIBPOSE_MODULE}
    ${OpenCV_LIBS}
)

# Hide console window on Windows system
if (WIN32)
  if (MSVC)
    set_target_properties(${LIBPOSEGUI_MODULE} PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
  elseif(CMAKE_COMPILER_IS_GNUCXX)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mwindows")
  endif()
endif()
