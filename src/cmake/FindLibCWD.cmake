IF ( WIN32 )
  SET ( WIN32_STYLE_FIND 1 )
ENDIF ()
IF ( MINGW )
  SET( WIN32_STYLE_FIND 0 )
  SET( UNIX_STYLE_FIND 1 )
ENDIF ()
IF ( UNIX )
  SET ( UNIX_STYLE_FIND 1 )
ENDIF ()

IF ( UNIX_STYLE_FIND )
  FIND_PATH ( LibCWD_INCLUDE_DIR libcwd/debug.h )
ELSEIF ( WIN32_STYLE_FIND )
  FIND_PATH ( LibCWD_INCLUDE_DIR libcwd/debug.h PATHS C:/ )
ENDIF ()
INCLUDE ( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS ( LibCWD DEFAULT_MSG LibCWD_INCLUDE_DIR )

