#ifndef MSGSYNCHRONIZER_H
#define MSGSYNCHRONIZER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mutex>
#include <functional>

typedef std::function<void(sensor_msgs::ImageConstPtr,std::vector<sensor_msgs::ImuConstPtr>)> msgCallback_t;

using namespace std;

namespace orb_slam2_vio
{
class MsgSynchronizer
{
public:
    enum Status{
        NOTINIT = 0,
        INIT,
        NORMAL
    };

    MsgSynchronizer(const double& imagedelay = 0.,const msgCallback_t &msgCallback = {});
    ~MsgSynchronizer();

    // add messages in callbacks
    void addImageMsg(const sensor_msgs::ImageConstPtr &imgmsg);
    void addImuMsg(const sensor_msgs::ImuConstPtr &imumsg);

    void clearMsgs();

    // for message callback if needed
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);

    //
    inline Status getStatus() {return _status;}

    double getImageDelaySec() const {return _imageMsgDelaySec;}

private:

    // check if messages are synced and return current image and
    // imu messages since last image
    bool getRecentMsgs(sensor_msgs::ImageConstPtr &imgmsg, std::vector<sensor_msgs::ImuConstPtr> &vimumsgs);

    double _imageMsgDelaySec;  // image message delay to imu message, in seconds
    std::mutex _mutexImageQueue;
    std::queue<sensor_msgs::ImageConstPtr> _imageMsgQueue;
    std::mutex _mutexIMUQueue;
    std::queue<sensor_msgs::ImuConstPtr> _imuMsgQueue;
    ros::Time _imuMsgTimeStart;
    Status _status;
    int _dataUnsyncCnt;

    // Callback function for when new message combinations are available
    msgCallback_t _msgCallback;
};

}

#endif // MSGSYNCHRONIZER_H
