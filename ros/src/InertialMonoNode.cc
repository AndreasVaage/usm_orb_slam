#include "InertialMonoNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        ROS_ERROR ("Path to vocabulary and path to settings need to be set.");
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR);

    ORB_SLAM2::ConfigParam config(argv[2]);

    ros::NodeHandle node_handle;

    image_transport::ImageTransport image_transport (node_handle);

    InertialMonoNode node (&SLAM, node_handle, image_transport, config);


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}


InertialMonoNode::InertialMonoNode (ORB_SLAM2::System* pSLAM, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::ConfigParam &config) :
Node (pSLAM, node_handle, image_transport),
_bAccMultiply98(config.GetAccMultiply9p8()),
_imageMsgDelaySec(config.GetImageDelayToIMU()),
_g(config.GetG()),
_msg_synchronizer(_imageMsgDelaySec, std::bind(&InertialMonoNode::MsgCallback, this, std::placeholders::_1,std::placeholders::_2)){

    _image_subscriber = image_transport.subscribe (config._imageTopic, 2, &orb_slam2_vio::MsgSynchronizer::imageCallback, &_msg_synchronizer);
    _imu_subscriber = node_handle.subscribe(config._imuTopic, config._imuRate, &orb_slam2_vio::MsgSynchronizer::imuCallback, &_msg_synchronizer);
}


InertialMonoNode::~InertialMonoNode () = default;

// Callback for when a valid combination of new image and imu msgs are available

void InertialMonoNode::MsgCallback (const sensor_msgs::ImageConstPtr& imgmsg,const std::vector<sensor_msgs::ImuConstPtr> &vimumsgs) {

    // Copy imu messages to IMUData
    std::vector<ORB_SLAM2::IMUData> vimuData;
    //ROS_INFO("image time: %.3f",imageMsg->header.stamp.toSec());
    for (const auto &imuMsg : vimumsgs)
    {
        double ax = imuMsg->linear_acceleration.x;
        double ay = imuMsg->linear_acceleration.y;
        double az = imuMsg->linear_acceleration.z;
        if(_bAccMultiply98)
        {
            ax *= _g;
            ay *= _g;
            az *= _g;
        }
        ORB_SLAM2::IMUData imudata(imuMsg->angular_velocity.x,imuMsg->angular_velocity.y,imuMsg->angular_velocity.z,
                                   ax,ay,az,imuMsg->header.stamp.toSec());
        vimuData.push_back(imudata);
        //ROS_INFO("imu time: %.3f",vimuMsg[i]->header.stamp.toSec());
    }

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_in_ptr;
    try {
      cv_in_ptr = cv_bridge::toCvShare(imgmsg);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


    current_frame_time_ = imgmsg->header.stamp;

    orb_slam_->TrackMonoVI(cv_in_ptr->image,vimuData,cv_in_ptr->header.stamp.toSec()-_imageMsgDelaySec);

    Update ();
}
