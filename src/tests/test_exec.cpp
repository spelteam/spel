// SPEL definitions
#include "predef.hpp"

#include <gtest/gtest.h>
#ifdef WINDOWS
#include <conio.h>
#include <string>
#endif  // WINDOWS
int main(int argc, char* argv[]) {
#if defined(MEMORY_DEBUG) && defined(UNIX)
  Debug(libcw_do.on());
  Debug(dc::malloc.on());
#endif  // MEMORY_DEBUG && UNIX
  testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();
#if defined(MEMORY_DEBUG) && defined(UNIX)
  Debug(list_allocations_on(libcw_do));
#endif  // MEMORY_DEBUG && UNIX
#ifdef WINDOWS
  if (argc > 1)
  {
    auto bPause = false;
    for (auto i = 0; i < argc; ++i)
      if (strcmp(argv[i], "-m") == 0)
      {
        bPause = true;
        break;
      }
    if (bPause)
      getch();
  }
#endif  // WINDOWS
  return res;
}

