#include <gtest/gtest.h>
#ifdef WINDOWS
#include <conio.h>
#endif  // WINDOWS
int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();
#ifdef WINDOWS
  getch();
#endif  // WINDOWS
  return res;
}

