/**
 * @file rectify_image.cpp
 * @brief This file contains the implementation of the rectify_image class.
 * @date 2024-08-14
 * @author Seungwook
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>


class RectifyImage
{
public:
    RectifyImage();
    ~RectifyImage();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::NodeHandle nh_, nhp_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber it_sub_;
    image_transport::Publisher it_pub_;

};

RectifyImage::RectifyImage(ros::NodeHandle nh, ros::NodeHandle nhp)
: nh_(nh), nhp_(nhp)
{
    it_ = image_transport::ImageTransport(nh);
    it_sub_ = it.subscribe("in_image_base_topic", 1, RectifyImage::imageCallback);
    it_pub_ = it.advertise("out_image_base_topic", 1);
}

RectifyImage::~RectifyImage()
{
}

void RectifyImage::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Do something
}