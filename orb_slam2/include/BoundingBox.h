//
// Created by andreas on 02.06.19.
//

#ifndef ORB_SLAM2_ROS_BOUNDINGBOX_H
#define ORB_SLAM2_ROS_BOUNDINGBOX_H

#include "usm_msgs/BoundingBox.h"

namespace ORB_SLAM2
{
class BoundingBox
{
private:
    std::string mClass;
    float mProb;
    int mXmin;
    int mYmin;
    int mXmax;
    int mYmax;
public:
    BoundingBox(const usm_msgs::BoundingBox &boundingBox):
        mClass(boundingBox.Class)
        ,mProb(boundingBox.probability)
        ,mXmin(boundingBox.xmin)
        ,mYmin(boundingBox.ymin)
        ,mXmax(boundingBox.xmax)
        ,mYmax(boundingBox.ymax)
    {}

    bool ContainsPoint(float x, float y, std::string &classification, float &prob) const
    {
      if (mXmin <= x and x <= mXmax and mYmin <= y and y <= mYmax)
      {
        classification = mClass;
        prob = mProb;
        return true;
      }
      return false;
    }
};
}

#endif //ORB_SLAM2_ROS_BOUNDINGBOX_H
