/**
 * @file rectify_image.cpp
 * @brief This file contains the implementation of the rectify_image class.
 * @date 2024-08-14
 * @author Seungwook
 */

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

class RectifyImage
{
public:
    RectifyImage(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~RectifyImage();
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::NodeHandle nh_, nhp_;
    ros::Subscriber camera_info_sub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber it_sub_;
    image_transport::Publisher it_pub_;

    sensor_msgs::CameraInfo camera_info_;
    image_geometry::PinholeCameraModel camera_model_;

    bool is_camera_model_set_;
    double rate_hz_, last_rect_time_;
    bool use_compressed_image_;
};

RectifyImage::RectifyImage(ros::NodeHandle nh, ros::NodeHandle nhp)
: nh_(nh), nhp_(nhp), it_(nh), is_camera_model_set_(false)
{
    rate_hz_ = nhp_.param<double>("rate_hz", 2.0);
    if(rate_hz_ <= 0)
    {
        ROS_WARN("rate_hz is not set properly(%f hz). Set to 2.0", rate_hz_);
        rate_hz_ = 2.0;
    }
    last_rect_time_ = ros::Time::now().toSec();

    use_compressed_image_ = nhp_.param<bool>("use_compressed_image", false);
    std::string hints = use_compressed_image_ ? "compressed" : "raw";
    it_sub_ = it_.subscribe("in_image_base_topic", 1, boost::bind(&RectifyImage::imageCallback, this, _1));
    it_pub_ = it_.advertise("out_image_base_topic", 1);
    camera_info_sub_ = nh_.subscribe("camera_info", 1, &RectifyImage::cameraInfoCallback, this);

    ROS_INFO("rectify_image node initialized.");
}

RectifyImage::~RectifyImage()
{
}

void RectifyImage::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    if(!is_camera_model_set_)
    {
        camera_info_ = *msg;
        camera_model_.fromCameraInfo(msg);
        is_camera_model_set_ = true;
        ROS_INFO("camera_info is set.");
        return;
    }
}

void RectifyImage::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if(!is_camera_model_set_)
    {
        ROS_WARN_THROTTLE(5,"camera_info is not set yet.");
        return;
    }
    else if(ros::Time::now().toSec() - last_rect_time_ < 1.0/rate_hz_)
    {
        return;
    }
    else
    {
        ROS_INFO_ONCE("Rectifying");
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image_rect;
        camera_model_.rectifyImage(cv_ptr->image, image_rect);
        cv_bridge::CvImage cv_image;
        cv_image.header = msg->header;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.image = image_rect;
        it_pub_.publish(cv_image.toImageMsg());
        ROS_INFO("Publishing rectified image.");
        last_rect_time_ = ros::Time::now().toSec();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rectify_image");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    RectifyImage rectify_image(nh, nhp);
    ros::spin();

    return 0;
}