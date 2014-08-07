#ifndef _MINSPANNINGTREE_HPP_
#define _MINSPANNINGTREE_HPP_

#include <vector>
#include "frame.hpp"
#include "imagesimilaritymatrix.hpp"
#include <opencv2/opencv.hpp>
#include <tree_util.hh>

using namespace std;
using namespace cv;

class MinSpanningTree
{
  public:
    MinSpanningTree();
    MinSpanningTree(const ImageSimilarityMatrix& ism, int rootNode, int treeSize, float threshold);
    MinSpanningTree(const MinSpanningTree& mst);
    ~MinSpanningTree();

    void build(const ImageSimilarityMatrix& ism, int rootNode, int treeSize, float threshold); //build the MST

    tree<int> getMST();

    //get size
    int size();

  private:
    tree<int> mst;
};

#endif  // _MINSPANNINGTREE_HPP_

