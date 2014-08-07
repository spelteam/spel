#include "minspanningtree.hpp"
#include <tree_util.hh>

using namespace kptree;

MinSpanningTree::MinSpanningTree(void)
{
	//constructor
}

MinSpanningTree::MinSpanningTree(const ImageSimilarityMatrix& ism, int rootNode, int treeSize, float threshold)
{
	build(ism, rootNode, treeSize, threshold);
}

MinSpanningTree::MinSpanningTree(const MinSpanningTree& _MST)
{
    mst = _MST.mst;
}

MinSpanningTree& MinSpanningTree::operator=(const MinSpanningTree& _MST)
{
    mst=_MST.mst;
    return *this;
}

MinSpanningTree::~MinSpanningTree(void)
{
	//destructor
}

tree<int> MinSpanningTree::getMST(void) const
{
	return mst;
}

//build an MST for a keyframe as root node of size treeSize and return it to caller
void MinSpanningTree::build(const ImageSimilarityMatrix& ism, int rootNode, int treeSize, float threshold)
{
    //initialise
    tree<int>::iterator imgLoc; //bone number, length, angle, root angle is with x axis
    imgLoc = mst.begin();

    vector<int> graphNodes; //nodes already in the graph
    graphNodes.push_back(rootNode);
    mst.insert(imgLoc, rootNode); //insert root node
    float absoluteMin = ism.mean();
    while(mst.size() != treeSize && mst.size()<ism.size()) //do this until the tree is complete
    {
        //cout << "in..." << endl;
        Point2i minLoc(-1,-1);
        float min=FLT_MAX;
        //for each node currently in the graph
        for(int i=0; i<graphNodes.size(); ++i)
        {
            for(int j=0; j<ism.size(); ++j)
            {
                int x=graphNodes[i];
                int y=j;
                //if y is not part of the graph yet
                if(std::find(graphNodes.begin(), graphNodes.end(), y) == graphNodes.end() && x!=y)
                {
                    float locScore = ism.at(x,y);
                    if(locScore<min)
                    {
                        min = locScore;
                        minLoc = Point2i(x,y);
                    }
                }
            }
            //find minimum connection node b not yet in graph
        }

        //insert minimum connection node into graph
        //minLoc and min should now contain the minimal values
        //insert into the tree as appropriate

        //set iterator to correct node (x) if there was anything to set
        if(min!=FLT_MAX)
        {
            for(imgLoc=mst.begin(); imgLoc!=mst.end(); ++imgLoc)
            {
                if(*imgLoc==minLoc.x)
                    break;
            }
            //check the path cost condition
            vector<int> path;
    		path = kptree::find_path_nodes(mst, mst.begin(), imgLoc);
            if(ism.getPathCost(path) > absoluteMin*threshold) //stop building tree if we reach threshold
                break;
            mst.append_child(imgLoc, minLoc.y);
            graphNodes.push_back(minLoc.y);
        }
    }
}

int MinSpanningTree::size() const
{
	
}