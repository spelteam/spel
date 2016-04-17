// SPEL definitions
#include "predef.hpp"

#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#include <gtest/gtest.h>
#include <tree_util.hh>

#include "solver.hpp"
#include "TestsFunctions.hpp"

namespace SPEL
{
  TEST(Solver, Constructor)
  {
    Solver solver;
    EXPECT_EQ(-1, solver.getId());
    EXPECT_EQ("BaseClass", solver.getName());
  }


}