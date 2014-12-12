#include <gtest/gtest.h>
#include <lockframe.hpp>

class LockframeTest : public testing::Test{
protected:
    //init
    virtual void SetUp(){
        //set id
        id = 7;

        //set image
        Point pt1, pt2;
        pt1.x = 10;
        pt1.y = 10;
        pt2.x = 90;
        pt2.y = 90;

        int icolor = 0x00FFAA11;
        Scalar color_image(icolor&255, (icolor>>8)&255, (icolor>>16)&255);

        image = Mat::zeros(100,100, CV_32F);
        line(image, pt1, pt2, color_image, 5, 8);

        //set mask
        pt1.x = 10;
        pt1.y = 90;
        pt2.x = 90;
        pt2.y = 10;

        icolor = 0x00AABBCC;
        Scalar color_mask(icolor&255, (icolor>>8)&255, (icolor>>16)&255);

        mask = Mat::zeros(100, 100, CV_32F);
        line(mask, pt1, pt2, color_mask, 9, 8);

        //set skeleton
        //nothing

        //set groupPoint
        groundPoint = Point2f(0.0,0.0);

        //create lockframe
        lockframe.setID(id);
        lockframe.setImage(image);
        lockframe.setMask(mask);
        lockframe.setSkeleton(skeleton);
        lockframe.setGroundPoint(groundPoint);

    }

    //clear
    //nothing to clear here
    /*virtual void TearDown(){}*/
protected:
    int id;
    Mat image;
    Mat mask;
    Skeleton skeleton;
    Point2f groundPoint;

    Lockframe lockframe;
};

TEST_F(LockframeTest, GetId){
    EXPECT_EQ( id, lockframe.getID() );
}

TEST_F( LockframeTest, GetImage ){
    EXPECT_EQ( 0, cv::countNonZero(image != lockframe.getImage()) );
    EXPECT_EQ( image.rows*image.cols, cv::countNonZero(image == lockframe.getImage()) );
}

TEST_F( LockframeTest, GetMask ){
    EXPECT_EQ( 0, cv::countNonZero(mask != lockframe.getMask()) );
    EXPECT_EQ( mask.rows*mask.cols, cv::countNonZero(mask == lockframe.getMask()) );
}

TEST_F( LockframeTest, GetSkeleton ){
    EXPECT_EQ( skeleton, lockframe.getSkeleton() );
}

TEST_F( LockframeTest, GetGroundPoint ){
    EXPECT_EQ( groundPoint, lockframe.getGroundPoint() );
}

TEST_F( LockframeTest, GetFrametype ){
    EXPECT_EQ( FRAMETYPE::LOCKFRAME, lockframe.getFrametype() );
}

