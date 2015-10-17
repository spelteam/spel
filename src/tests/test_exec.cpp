#include <gtest/gtest.h>
#ifdef WINDOWS
#include <conio.h>
#include <string>
#endif  // WINDOWS
#include "memoryDebug.hpp"
int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();

#ifdef DEBUG
  DumpUnfreed();
#endif // DEBUG

#ifdef WINDOWS
  if (argc == 2 && strcmp(argv[1], "-m") == 0)
    getch();
#endif  // WINDOWS
  return res;
}

