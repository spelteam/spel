# spelgui module
# Creates a spel graphic user interface 
cmake_minimum_required(VERSION 2.8.11)

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
# Add spel library
INCLUDE_DIRECTORIES ( "${${PROJECT_NAME}_SOURCE_DIR}/${SPEL_MODULE}" )

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
LIST(APPEND ${SPELGUI_MODULE}_SRC main.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC mainwindow.cpp)
#Forms
LIST(APPEND ${SPELGUI_MODULE}_SRC solverparametersdialog.cpp)
#Custom layouts
LIST(APPEND ${SPELGUI_MODULE}_SRC flowlayout.cpp)
#Custom group boxes
LIST(APPEND ${SPELGUI_MODULE}_SRC frameboxwidget.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC solveboxwidget.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC toolboxwidget.cpp)
#Custom table widget
LIST(APPEND ${SPELGUI_MODULE}_SRC frametablewidget.cpp)
#View helper
LIST(APPEND ${SPELGUI_MODULE}_SRC frameview2d.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC frameview3d.cpp)
#Project file
LIST(APPEND ${SPELGUI_MODULE}_SRC project.cpp)
#Utility helper class
LIST(APPEND ${SPELGUI_MODULE}_SRC utility.cpp)
#Custom graphick items
LIST(APPEND ${SPELGUI_MODULE}_SRC bodyjointitem.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC bodypartitem.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC limblabelitem.cpp)
#Xml message handler
LIST(APPEND ${SPELGUI_MODULE}_SRC xmlmessagehandler.cpp)
#Attributes of xml project
LIST(APPEND ${SPELGUI_MODULE}_SRC projectattr.cpp)
#Documents
LIST(APPEND ${SPELGUI_MODULE}_SRC projectdocument.cpp)
#Model handlers
LIST(APPEND ${SPELGUI_MODULE}_SRC model_handlers/bodyjointhandler.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC model_handlers/bodyparthandler.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC model_handlers/bodyjointheaderhandler.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC model_handlers/bodypartheaderhandler.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC model_handlers/framehandler.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC model_handlers/projecthandler.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC model_handlers/bodyjointsheaderhandler.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC model_handlers/bodypartsheaderhandler.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC model_handlers/bodyjointshandler.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC model_handlers/bodypartshandler.cpp)
LIST(APPEND ${SPELGUI_MODULE}_SRC model_handlers/frameshandler.cpp)
LIST(APPEND ${SPELGUI_MODULE}_HDR model_handlers/modelhandler.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR model_handlers/bodyjointhandler.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR model_handlers/bodyparthandler.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR model_handlers/bodyjointheaderhandler.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR model_handlers/bodypartheaderhandler.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR model_handlers/framehandler.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR model_handlers/projecthandler.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR model_handlers/bodyjointsheaderhandler.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR model_handlers/bodypartsheaderhandler.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR model_handlers/bodyjointshandler.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR model_handlers/bodypartshandler.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR model_handlers/frameshandler.h)
#Transitional classes
LIST(APPEND ${SPELGUI_MODULE}_HDR frameattrstorage.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR projectattrstorage.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR filepathstorage.h)
#Exceptions
LIST(APPEND ${SPELGUI_MODULE}_HDR exceptions.h)
#Other
LIST(APPEND ${SPELGUI_MODULE}_HDR bodyjointitem.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR bodypartitem.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR flowlayout.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR frameboxwidget.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR frametablewidget.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR frameview2d.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR frameview3d.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR limblabelitem.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR mainwindow.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR speldocument.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR project.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR projectattr.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR projectdocument.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR solveboxwidget.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR solverparametersdialog.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR toolboxwidget.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR utility.h)
LIST(APPEND ${SPELGUI_MODULE}_HDR xmlmessagehandler.h)


#Add forms
QT5_WRAP_UI(UIS_HDRS
    mainwindow.ui
    solverparametersdialog.ui
)

#Add resources files
QT5_ADD_RESOURCES( ${SPELGUI_MODULE}_RRC resources.qrc )


# Tell CMake to create the executable
add_executable(${SPELGUI_MODULE} ${${SPELGUI_MODULE}_SRC} ${${SPELGUI_MODULE}_HDR} ${UIS_HDRS} ${${SPELGUI_MODULE}_RRC} )

if (WIN32)
  set (QtMain Qt5::WinMain)
endif()

IF ( UNIX )
  SET ( GUID_LNK uuid )
ELSEIF ( WIN32 )
  SET ( GUID_LNK rpcrt4 )
ENDIF()

# Add libraries
target_link_libraries( ${SPELGUI_MODULE}
    ${QtMain}
    Qt5::Core
    Qt5::Widgets
    Qt5::Xml
    Qt5::XmlPatterns
    Qt5::OpenGL
    ${SPEL_MODULE}
    ${OpenCV_LIBS}
    ${HDF5_LIBRARIES}
    ${LibCWD_LIBRARIES}
    ${GUID_LNK}
)

# Hide console window on Windows system
if (WIN32)
  if (MSVC)
    set_target_properties(${SPELGUI_MODULE} PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
  elseif(CMAKE_COMPILER_IS_GNUCXX)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mwindows")
  endif()
endif()

IF ( UNIX )
	INSTALL (
		TARGETS ${SPELGUI_MODULE}
		RUNTIME DESTINATION bin/
		LIBRARY DESTINATION lib/${PROJECT_NAME}
		ARCHIVE DESTINATION lib/static/${PROJECT_NAME}
	)
ELSEIF ( WIN32 )
	INSTALL (
		TARGETS ${SPELGUI_MODULE}
		RUNTIME DESTINATION bin/
		LIBRARY DESTINATION lib/
		ARCHIVE DESTINATION lib/static/
	)
ENDIF()

