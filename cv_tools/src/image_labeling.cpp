//
// Created by Petro Shmigelskyi on 10/29/22.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/String.h>


const std::string IMAGE_TOPIC = "video_stream";
const std::string IMAGE_LABELS_TOPIC = "image_labels";
const double PROCESSING_TIME = 0.03; // seconds


class ImageLabelingNode {
public:
    ImageLabelingNode();
    virtual ~ImageLabelingNode();

protected:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber image_sub;
    ros::Publisher* labels_pub;

    void imageCallback(const sensor_msgs::CompressedImageConstPtr &msg);
};

ImageLabelingNode::ImageLabelingNode()
:nh(),
pnh("~")
{
    // subscribe to image topics
    image_sub = nh.subscribe(IMAGE_TOPIC, 5, &ImageLabelingNode::imageCallback, this);
    // create labeling metadata publisher
    labels_pub = new ros::Publisher(nh.advertise<std_msgs::String>(IMAGE_LABELS_TOPIC, 5));
}

ImageLabelingNode::~ImageLabelingNode()
{
    delete labels_pub;
}

void ImageLabelingNode::imageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    ROS_INFO_NAMED("image_labeling", "CV: Image received.");

    // simulate processing time
    ros::Duration(PROCESSING_TIME).sleep();

    std_msgs::String resMsg;
    resMsg.data = std::to_string(msg->header.seq);//"{“id”: id, “box”: [center_x, center_y, width, height]}";
    labels_pub->publish(resMsg);
    ROS_INFO_NAMED("image_labeling", "CV: Image processed. Labels published.");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_labeling_node");

    ImageLabelingNode labelingNode;

    ros::spin();

    return 0;
}