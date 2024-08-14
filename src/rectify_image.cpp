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
#include <compressed_image_transport/compressed_subscriber.h>
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
    image_geometry::PinholeCameraModel camera_model_;
    // compressed_image_transport::CompressedSubscriber it_compressed_sub_;
    bool is_camera_model_set_;
    sensor_msgs::CameraInfo camera_info_;
};

RectifyImage::RectifyImage(ros::NodeHandle nh, ros::NodeHandle nhp)
: nh_(nh), nhp_(nhp), it_(nh)
{
    it_sub_ = it_.subscribe("in_image_base_topic", 1, boost::bind(&RectifyImage::imageCallback, this, _1));
    // it_compressed_sub_ = compressed_image_transport::CompressedSubscriber(nh_);
    it_pub_ = it_.advertise("out_image_base_topic", 1);
    camera_info_sub_ = nh_.subscribe("camera_info", 1, &RectifyImage::cameraInfoCallback, this);
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
        return;
    }
}

void RectifyImage::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if(!is_camera_model_set_)
    {
        ROS_WARN_THROTTLE(5,"Camera model is not set yet.");
        return;
    }
    else
    {
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