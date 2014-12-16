#include <gtest/gtest.h>
#include <keyframe.hpp>

class KeyframeTest : public testing::Test{
protected:
    //init
    //nothing to init here
    /*virtual void SetUp(){}*/

    //clear
    //nothing to clear here
    /*virtual void TearDown(){}*/
protected:
    Keyframe keyframe;
};

TEST_F( KeyframeTest, GetFrametype ){
    EXPECT_EQ( FRAMETYPE::KEYFRAME, keyframe.getFrametype() );
}

