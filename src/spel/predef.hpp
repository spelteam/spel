#ifndef _PREDEF_H_
#define _PREDEF_H_

#undef SPEL_MAJOR_VERSION
#undef SPEL_MINOR_VERSION
#undef SPEL_PATCH_LEVEL
#undef SPEL_REVISION
#undef SPEL_VERSION

#define SPEL_MAJOR_VERSION 1
#define SPEL_MINOR_VERSION 1
#define SPEL_PATCH_LEVEL 0
#define SPEL_REVISION 0
#define SPEL_VERSION \
  SPEL_MAJOR_VERSION.SPEL_MINOR_VERSION.SPEL_PATCH_LEVEL.SPEL_REVISION

#define NOMINMAX

#include "memoryDebug.hpp"

#endif  // _PREDEF_H_
