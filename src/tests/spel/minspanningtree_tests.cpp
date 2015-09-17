#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#include <gtest/gtest.h>
#include <tree_util.hh>

#include "frame.hpp"
#include "imagesimilaritymatrix.hpp"
#include "minspanningtree.hpp"
#include "TestsFunctions.hpp"

TEST(MinSpanningTree, build)
{
  //Read testing ISM matrix
  ImageSimilarityMatrix ISM;
  bool b;
  string FilePath = "speltests_TestData/SimilarityMatrixTestsData/";
#if defined(WINDOWS) && defined(_MSC_VER)
  if (IsDebuggerPresent())
    FilePath = "Debug/" + FilePath;
#endif
  b = ISM.read(FilePath + "ISM.txt");
  ASSERT_TRUE(b);

  //Build expected minspanningtree
  tree<int> expected_mst;
  int rootNode = 3;

  tree<int>::iterator A = expected_mst.begin(), B;
  A = expected_mst.insert(A, rootNode);
  B = expected_mst.append_child(A, 0);
  expected_mst.append_child(A, 5);
  B = expected_mst.append_child(B, 1);
  B = expected_mst.append_child(B, 4);
  expected_mst.append_child(B, 2);
  expected_mst.append_child(B, 6);

  //Create actual MST
  MinSpanningTree MST;
  MST.build(ISM, rootNode, ISM.size(), 0.4f);
  tree<int> actual_mst = MST.getMST();

  //Compare
  ASSERT_EQ(expected_mst.size(), actual_mst.size());
  tree<int>::iterator actual_node = actual_mst.begin();
  for (tree<int>::iterator expected_node = expected_mst.begin(); expected_node != expected_mst.end(); ++expected_node)
  {
    EXPECT_EQ(*expected_node, *actual_node);
    //cout << *expected_node << "-" << *actual_node << endl;
    actual_node++;
  }

  actual_mst.clear();
  expected_mst.clear();
}

TEST(MinSpanningTree, MinSpanningTree)
{
  //Read testing ISM matrix
  ImageSimilarityMatrix ISM;
  bool b;
  string FilePath = "speltests_TestData/SimilarityMatrixTestsData/";
#if defined(WINDOWS) && defined(_MSC_VER)
  if (IsDebuggerPresent())
    FilePath = "Debug/" + FilePath;
#endif
  b = ISM.read(FilePath + "ISM.txt");
  ASSERT_TRUE(b);

  //Build expected minspanningtree
  tree<int> expected_mst;
  int rootNode = 3;

  tree<int>::iterator A = expected_mst.begin(), B;
  A = expected_mst.insert(A, rootNode);
  B = expected_mst.append_child(A, 0);
  expected_mst.append_child(A, 5);
  B = expected_mst.append_child(B, 1);
  B = expected_mst.append_child(B, 4);
  expected_mst.append_child(B, 2);
  expected_mst.append_child(B, 6);

  //Create actual MST
  MinSpanningTree MST(ISM, rootNode, ISM.size(), 1.0f);
  tree<int> actual_mst = MST.getMST();

  //Compare
  ASSERT_EQ(expected_mst.size(), actual_mst.size());
  tree<int>::iterator actual_node = actual_mst.begin();
  for (tree<int>::iterator expected_node = expected_mst.begin(); expected_node != expected_mst.end(); ++expected_node)
  {
    EXPECT_EQ(*expected_node, *actual_node);
    //cout << *expected_node << "-" << *actual_node << endl;
    actual_node++;
  }

  actual_mst.clear();
  expected_mst.clear();
}