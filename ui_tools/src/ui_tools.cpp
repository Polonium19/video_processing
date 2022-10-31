//
// Created by Petro Shmigelskyi on 10/29/22.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

const std::string IMAGE_TOPIC = "video_stream";

void imageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    ROS_INFO_NAMED("ui_tools", "UI: Image received");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ui_tools");
    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe(IMAGE_TOPIC, 1, imageCallback);

    ros::spin();

    return 0;
}