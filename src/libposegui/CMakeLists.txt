# libposegui module
# Creates a libpose graphic user interface 
cmake_minimum_required(VERSION 2.8.11)

#project(LibposeGUI)

# Configure modules path
SET ( MODULE_PATH "${${PROJECT_NAME}_SOURCE_DIR}/cmake" )
INCLUDE ( "${MODULE_PATH}/modules.cmake" )

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# Find the QtWidgets library
find_package(Qt5Widgets)

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

#Add forms
QT5_WRAP_UI(UIS_HDRS
    mainwindow.ui
)

#Add resources files
QT5_ADD_RESOURCES( ${LIBPOSEGUI_MODULE}_RRC resources.qrc )


# Tell CMake to create the helloworld executable
add_executable(${LIBPOSEGUI_MODULE} ${${LIBPOSEGUI_MODULE}_SRC} ${UIS_HDRS} ${${LIBPOSEGUI_MODULE}_RRC} )

# Use the Widgets module from Qt 5.
target_link_libraries(${LIBPOSEGUI_MODULE} Qt5::Widgets)
