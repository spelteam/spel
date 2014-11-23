if(WIN32)
  set(WIN32_STYLE_FIND 1)
endif()
if(MINGW)
  set(WIN32_STYLE_FIND 0)
  set(UNIX_STYLE_FIND 1)
endif()
if(UNIX)
  set(UNIX_STYLE_FIND 1)
endif()


if (UNIX_STYLE_FIND)
FIND_PATH ( OpenGM_INCLUDE_DIR opengm/opengm.hxx )

INCLUDE ( FindPackageHandleStandardArgs )

FIND_PACKAGE_HANDLE_STANDARD_ARGS ( OpenGM DEFAULT_MSG OpenGM_INCLUDE_DIR )



endif()

if(WIN32_STYLE_FIND)

FIND_PATH ( OpenGM_INCLUDE_DIR opengm/opengm.hxx
 PATHS
  C:/opengm-2.3.3)

  #message("found OpenGM_INCLUDE_DIR: ${OpenGM_INCLUDE_DIR}")
 #set (OpenGM_INCLUDE_DIR ${OpenGM_INCLUDE_DIR})
INCLUDE ( FindPackageHandleStandardArgs )

FIND_PACKAGE_HANDLE_STANDARD_ARGS ( OpenGM DEFAULT_MSG OpenGM_INCLUDE_DIR )



endif()
