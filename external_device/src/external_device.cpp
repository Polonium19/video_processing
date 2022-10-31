//
// Created by Petro Shmigelskyi on 10/29/22.
//

#include "ros/ros.h"
#include "std_msgs/String.h"

const std::string IMAGE_LABELS_TOPIC = "image_labels";


void imageLabelsCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO_NAMED("external_device", "DEVICE: Image labeling received.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "external_device");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe(IMAGE_LABELS_TOPIC, 5, imageLabelsCallback);

    ros::spin();

    return 0;
}