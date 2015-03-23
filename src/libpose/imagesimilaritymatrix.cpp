#include "imagesimilaritymatrix.hpp"
#include <fstream>

ImageSimilarityMatrix::ImageSimilarityMatrix(void)
{
    //nothing to do
}
ImageSimilarityMatrix::~ImageSimilarityMatrix(void)
{

}

ImageSimilarityMatrix::ImageSimilarityMatrix(const vector<Frame*>& frames)
{
    //by default use colour
    buildImageSimilarityMatrix(frames);
}

ImageSimilarityMatrix::ImageSimilarityMatrix(const ImageSimilarityMatrix& m)
{
    //by default use colour
    imageSimilarityMatrix=m.imageSimilarityMatrix;
}

//get ISM value at (row, col)
float ImageSimilarityMatrix::at(int row, int col) const
{

    if(row>=imageSimilarityMatrix.rows)
    {
        cerr << "ISM contains " << imageSimilarityMatrix.rows << " rows, cannot request row " << endl; //<< to_string(row) << end;
        return -1;
    }
    if(col>=imageSimilarityMatrix.cols)
    {
        cerr << "ISM contains " << imageSimilarityMatrix.cols << " cols, cannot request col " << endl; // << to_string(col) << end;
        return -1;
    }
    return imageSimilarityMatrix.at<float>(row,col);
}

bool ImageSimilarityMatrix::operator==(const ImageSimilarityMatrix &s) const
{
    Mat result = (imageSimilarityMatrix==s.imageSimilarityMatrix);
    
    bool res=true;

    //if every element is 1
    for(int i=0; i<result.rows; ++i)
    {
        for(int j=0; j<result.cols; ++j)
        {
            if(result.at<float>(i,j)==0)
                res=false;
        }
    }
    return res;
}

bool ImageSimilarityMatrix::operator!=(const ImageSimilarityMatrix &s) const
{
    Mat result = (imageSimilarityMatrix==s.imageSimilarityMatrix);
    
    bool res=true;

    //if every element is 1
    for(int i=0; i<result.rows; ++i)
    {
        for(int j=0; j<result.cols; ++j)
        {
            if(result.at<float>(i,j)==0)
                res=false;
        }
    }
    return !res;
}

ImageSimilarityMatrix& ImageSimilarityMatrix::operator=(const ImageSimilarityMatrix &s)
{
    imageSimilarityMatrix=s.imageSimilarityMatrix;
    return *this;
}

bool ImageSimilarityMatrix::read(string filename)
{
    ifstream in(filename.c_str());
    if(in.is_open())
    {
        for(int i=0; i<imageSimilarityMatrix.rows; ++i)
        {
            for(int j=0; j<imageSimilarityMatrix.cols; ++j)
            {
                float score;
                in >>  score;
                imageSimilarityMatrix.at<float>(i,j)=score;
            }
        }
        return true;
    }
    else
    {
        cerr << "Could not open " << filename << " for reading. " << endl;
        return false;
    }
}

bool ImageSimilarityMatrix::write(string filename) const
{
    ofstream out(filename.c_str());
    if(out.is_open())
    {
        for(int i=0; i<imageSimilarityMatrix.rows; ++i)
        {
            for(int j=0; j<imageSimilarityMatrix.cols; ++j)
            {
                out << imageSimilarityMatrix.at<float>(i,j) << " ";
            }
            out << endl;
        }
        return true;
    }
    else
    {
        cerr << "Could not open " << filename << " for writing. " << endl;
        return false;
    }
}
void ImageSimilarityMatrix::buildMaskSimilarityMatrix(const vector<Frame*>& frames)
{
     //create matrices and fill with zeros
    // imageSimilarityMatrix.create(frames.size(), frames.size(), DataType<float>::type);
    imageSimilarityMatrix.create(frames.size(), frames.size(), DataType<float>::type);

    for(uint32_t i=0; i<frames.size(); ++i)
    {
        for(uint32_t j=0; j<frames.size(); ++j)
        {
            // imageSimilarityMatrix.at<float>(i,j) = 0;
            imageSimilarityMatrix.at<float>(i,j) = 0;
        }
    }

    //compute mask centroid offsets
    for(uint32_t i=0; i<frames.size(); ++i)
    {
        for(uint32_t j=0; j<frames.size(); ++j)
        {
            //load images, compute similarity, store to matrix
            // Mat imgMatOne=frames[i]->getImage();
            // Mat imgMatTwo=frames[j]->getImage());

            Mat maskMatOne=frames[i]->getMask();
            Mat maskMatTwo=frames[j]->getMask();

            Point2f cOne, cTwo;
            float mSizeOne=0, mSizeTwo=0;

            // cout << "at " << to_string(i) << ", " << to_string(j) << " ";
            //compute similarity score
//            if(i==j)
//                cout<<"SPECIAL CASE" << endl;

            //compute deltaX from centroid
            Point2f dX;
            for(int x=0; x<maskMatOne.rows; ++x)
            {
                for(int y=0; y<maskMatOne.cols; ++y)
                {
                    int intensity = maskMatOne.at<uchar>(y, x);
                    int mintensity = maskMatTwo.at<uchar>(y, x);

                    bool darkPixel=intensity<10;
                    bool blackPixel=mintensity<10; //if all intensities are zero

                    if(!darkPixel) //if the pixel is non-black for maskOne
                    {
                        //pixOne = imgOne.pixel(x,y);
                        cOne+=Point2f(x,y);
                        mSizeOne++;
                    }

                    if(!blackPixel) //if the pixel is non-black for maskOne
                    {
                        //pixTwo = imgTwo.pixel(x,y);
                        cTwo+=Point2f(x,y);
                        mSizeTwo++;
                    }

                }
            }
            cOne = cOne*(1.0/mSizeOne);
            cTwo = cTwo*(1.0/mSizeTwo);

            //cOne and cTwo now have the centres
            dX = cTwo-cOne;

            //so, dX+cOne = cTwo
            //and cOne = cTwo-dX

            // float similarityScore = 0;
            float maskSimilarityScore = 0;
            for(int x=0; x<maskMatOne.rows; ++x)
            {
                for(int y=0; y<maskMatOne.cols; ++y)
                {
                    int mintensityOne = maskMatOne.at<uchar>(j, i);

                    bool darkPixel=mintensityOne<10; //if all intensities are zero

                    //QColor pixOne;
                    //QColor pixTwo;
                    int mOne=0, mTwo=0;

                    //apply the transformation
                    int xTwo = x+dX.x;
                    int yTwo = y+dX.y;

                    int mintensityTwo = maskMatTwo.at<uchar>(yTwo, xTwo);

                    bool blackPixel=mintensityTwo<10; //if all intensities are zero

                    // int blueOne;
                    // int greenOne;
                    // int redOne;

                    // int blueTwo;
                    // int greenTwo;
                    // int redTwo;

                    //compare points
                    if(!darkPixel)
                    {
                        mOne = 1;
                        // Vec4b intensityOne = imgMatOne.at<Vec4b>(y, x);
                        // blueOne = intensityOne.val[0];
                        // greenOne = intensityOne.val[1];
                        // redOne = intensityOne.val[2];
                    }
                    if(xTwo<maskMatTwo.rows && xTwo >=0 && yTwo < maskMatTwo.cols && yTwo >= 0 && !blackPixel)
                    {
                        mTwo = 1;
                        // Vec4b intensityTwo = imgMatTwo.at<Vec4b>(j, i);
                        // blueTwo = intensityTwo.val[0];
                        // greenTwo = intensityTwo.val[1];
                        // redTwo = intensityTwo.val[2];
                    }

                    maskSimilarityScore+=abs(mOne-mTwo);

                    // if(mOne^mTwo) //maximum penalty if they are different
                    // {
                    //     similarityScore+=pow(255, 2)+pow(255, 2)+pow(255, 2); //square of absolute difference
                    // }
                    // else //if both are in mask, or outside of mask
                    // {
                    //     similarityScore+=pow(redOne-redTwo, 2)+pow(greenOne-greenTwo, 2)+
                    //         pow(blueOne-blueTwo, 2); //square of absolute difference
                    // }
                }
            }

            // cout << " score = " << maskSimilarityScore << endl;

            // imageSimilarityMatrix.at<float>(i,j) = similarityScore;
            // imageSimilarityMatrix.at<float>(j,i) = similarityScore;
            imageSimilarityMatrix.at<float>(i,j) = maskSimilarityScore;
            imageSimilarityMatrix.at<float>(j,i) = maskSimilarityScore;
        }
    }
    return;
}

void ImageSimilarityMatrix::buildImageSimilarityMatrix(const vector<Frame*>& frames)
{
    cerr << "building ISM matrix" <<endl;
    //create matrices and fill with zeros
    imageSimilarityMatrix.create(frames.size(), frames.size(), DataType<float>::type);
    // maskSimilarityMatrix.create(frames.size(), frames.size(), DataType<float>::type);

    for(uint32_t i=0; i<frames.size(); ++i)
    {
        for(uint32_t j=0; j<frames.size(); ++j)
        {
            imageSimilarityMatrix.at<float>(i,j) = 0;
            // maskSimilarityMatrix.at<float>(i,j) = 0;
        }
    }

    //compute mask centroid offsets
    for(uint32_t i=0; i<frames.size(); ++i)
    {
        for(uint32_t j=0; j<frames.size(); ++j)
        {
            //load images, compute similarity, store to matrix
            Mat imgMatOne=frames[i]->getImage();
            Mat imgMatTwo=frames[j]->getImage();

            Mat maskMatOne=frames[i]->getMask();
            Mat maskMatTwo=frames[j]->getMask();

            Point2f cOne, cTwo;
            float mSizeOne=0, mSizeTwo=0;

            // cout << "at " << i << ", " << j << " ";

            Point2f dX;
            for(int x=0; x<maskMatOne.rows; ++x)
            {
                for(int y=0; y<maskMatOne.cols; ++y)
                {
                    Scalar intensity = maskMatOne.at<uchar>(x, y);
                    Scalar mintensity = maskMatTwo.at<uchar>(x, y);

                    bool darkPixel=intensity.val[0]<10;
                    bool blackPixel=mintensity.val[0]<10; //if all intensities are zero

                    if(!darkPixel) //if the pixel is non-black for maskOne
                    {
                        //pixOne = imgOne.pixel(x,y);
                        cOne+=Point2f(x,y);
                        mSizeOne++;
                    }

                    if(!blackPixel) //if the pixel is non-black for maskOne
                    {
                        //pixTwo = imgTwo.pixel(x,y);
                        cTwo+=Point2f(x,y);
                        mSizeTwo++;
                    }

                }
            }
            cOne = cOne*(1.0/mSizeOne);
            cTwo = cTwo*(1.0/mSizeTwo);

            //cOne and cTwo now have the centres
            dX = cTwo-cOne;

            //so, dX+cOne = cTwo
            //and cOne = cTwo-dX


            float similarityScore = 0;
            // float maskSimilarityScore = 0;
            for(int x=0; x<maskMatOne.rows; ++x)
            {
                for(int y=0; y<maskMatOne.cols; ++y)
                {
                    int mintensityOne = maskMatOne.at<uchar>(x, y);

                    bool darkPixel=mintensityOne<10; //if all intensities are zero

                    //QColor pixOne;
                    //QColor pixTwo;
                    int mOne=0, mTwo=0;

                    //apply the transformation
                    int xTwo = x+dX.x;
                    int yTwo = y+dX.y;

                    //now check bounds

                    int blueOne;
                    int greenOne;
                    int redOne;

                    int blueTwo;
                    int greenTwo;
                    int redTwo;

                    //compare points
                    if(!darkPixel)
                    {
                        mOne = 1;
                        Vec4b intensityOne = imgMatOne.at<Vec4b>(x, y);
                        blueOne = intensityOne.val[0];
                        greenOne = intensityOne.val[1];
                        redOne = intensityOne.val[2];
                    }
                    if(xTwo<imgMatTwo.rows && xTwo >=0 && yTwo < imgMatTwo.cols && yTwo >= 0)
                    {
                        Scalar mintensityTwo = maskMatTwo.at<uchar>(xTwo, yTwo);
                        bool blackPixel=mintensityTwo.val[0]<10; //if all intensities are zero

                        if(!blackPixel)
                        {
                            mTwo = 1;
                            Vec4b intensityTwo = imgMatTwo.at<Vec4b>(xTwo, yTwo);
                            blueTwo = intensityTwo.val[0];
                            greenTwo = intensityTwo.val[1];
                            redTwo = intensityTwo.val[2];
                        }
                    }

                    // maskSimilarityScore+=abs(mOne-mTwo);

                    if(mOne^mTwo) //maximum penalty if they are different
                    {
                        similarityScore+=pow(255, 2)+pow(255, 2)+pow(255, 2); //square of absolute difference
                    }
                    else //if both are in mask, or outside of mask
                    {
                        similarityScore+=pow(redOne-redTwo, 2)+pow(greenOne-greenTwo, 2)+
                            pow(blueOne-blueTwo, 2); //square of absolute difference
                    }
                }
            }

            // cout << " score = " << similarityScore << endl;

            imageSimilarityMatrix.at<float>(i,j) = similarityScore;
            imageSimilarityMatrix.at<float>(j,i) = similarityScore;
            // maskSimilarityMatrix.at<float>(i,j) = maskSimilarityScore;
            // maskSimilarityMatrix.at<float>(j,i) = maskSimilarityScore;
        }
    }
    return;
}

float ImageSimilarityMatrix::min() const//find the non-zero minimum in the image similarity matrix
{
    float min = FLT_MAX;
    for(int i=0; i<imageSimilarityMatrix.rows; ++i)
    {
        for(int j=0; j<imageSimilarityMatrix.cols; ++j)
        {
            float val = imageSimilarityMatrix.at<float>(i,j);
            if(val!=0 && val<min && i!=j)
                min=val;
        }
    }
    // cout << "THE MINUMUM FOR THIS ISM IS: " << min << endl;
    return min;
}

float ImageSimilarityMatrix::max() const//find the non-zero minimum in the image similarity matrix
{
    float max = -1;
    for(int i=0; i<imageSimilarityMatrix.rows; ++i)
    {
        for(int j=0; j<imageSimilarityMatrix.cols; ++j)
        {
            float val = imageSimilarityMatrix.at<float>(i,j);
            if(val>max)
                max=val;
        }
    }

    return max;
}

float ImageSimilarityMatrix::mean() const//find the non-zero minimum in the image similarity matrix
{
    float sum=0;
    float count=0;
    for(int i=0; i<imageSimilarityMatrix.rows; ++i)
    {
        for(int j=0; j<imageSimilarityMatrix.cols; ++j)
        {
            float val = imageSimilarityMatrix.at<float>(i,j);
            if(val!=0 && i!=j)
            {
                count++;
                sum+=val;
            }
        }
    }
    float mean = sum/count;
    // cout << "THE MEAN FOR THIS ISM IS: " << mean << endl;
    return mean;
}

float ImageSimilarityMatrix::getPathCost(vector<int> path) const//get cost for path through ISM
{
    //check that the path is valid
    for(uint32_t i=0; i<path.size();++i)
    {
        if(!(path[i]<imageSimilarityMatrix.rows))
        {
            cerr << "Path contains invalid node " << path[i] << endl;
            return -1;
        }
    }
    float cost=0;
    for(uint32_t i=1; i<path.size(); ++i) //get the cost from previous node to this node to the end
    {
        cost+=imageSimilarityMatrix.at<float>(path[i-1], path[i]);
    }
    return cost;
}

//return the size of the ISM
uint32_t ImageSimilarityMatrix::size() const
{
    return imageSimilarityMatrix.rows;
}
