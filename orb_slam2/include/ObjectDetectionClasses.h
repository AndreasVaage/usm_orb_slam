
#ifndef ORB_SLAM2_ROS_OBJECTDETECTIONCLASSES_H
#define ORB_SLAM2_ROS_OBJECTDETECTIONCLASSES_H


#include <algorithm>
#include <map>


namespace ORB_SLAM2
{
struct MapPointClassification
{
  string classification;
  double percentage;
};

class ObjectDetectionClasses
{
public:
    ObjectDetectionClasses()
    {
      nTotalClassifications = 0;
      mScores = {{"background",0}};
    }

    MapPointClassification GetBestClassification()
    {
      auto it = std::max_element(mScores.begin(),mScores.end(),[] (const std::pair<string,int>& a, const std::pair<string,int>& b)->bool{ return a.second < b.second; } );
      MapPointClassification result;
      result.classification = it->first;
      result.percentage = it->second / (double)nTotalClassifications;
      return result;
    }
    void AddClassification(const string &classification)
    {
      mScores[classification]++;
      nTotalClassifications++;
    }

  private:
    std::map<string,int> mScores;
    int nTotalClassifications;
};
}
#endif //ORB_SLAM2_ROS_OBJECTDETECTIONCLASSES_H
