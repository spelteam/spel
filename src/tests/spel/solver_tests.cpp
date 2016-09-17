// SPEL definitions
#include "predef.hpp"

#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#include <gtest/gtest.h>
#include <tree_util.hh>

#include "nskpsolver.hpp"
#include "TestsFunctions.hpp"

namespace SPEL
{
  TEST(Solver, Constructor)
  {
    Solver *solver = new NSKPSolver();
    EXPECT_NE(-1, solver->getId());
    EXPECT_NE("BaseClass", solver->getName());
  }


}