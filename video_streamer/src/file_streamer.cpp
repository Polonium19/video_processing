//
// Created by Petro Shmigelskyi on 10/29/22.
//

#include <ros/ros.h>

// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;


const std::string VIDEO_FILE = "";
const std::string VIDEO_TOPIC = "video_stream";
const int FPS_RATE = 25;


int main(int argc, char **argv)
{

    ros::init(argc, argv, "video_file_streamer");
    ros::NodeHandle nh, pnh("~");

    ros::Publisher image_pub = nh.advertise<sensor_msgs::CompressedImage>(VIDEO_TOPIC, 5);

    std::string filePath;
    pnh.param<std::string>("file", filePath, VIDEO_FILE);


    ros::Rate loop_rate(FPS_RATE);

    VideoCapture video(filePath);
    if(!video.isOpened()) {
        ROS_ERROR_NAMED("Video file streamer", "File not found: %s", filePath.c_str());
        return -1;
    }

    while(image_pub.getNumSubscribers() == 0) {
        ROS_INFO_STREAM("Waiting for subscribers");
        ros::Duration(1).sleep();
    }

    ROS_INFO_NAMED("Video file streamer", "Start streaming");

    cv_bridge::CvImage cvImage;
    cvImage.encoding = "bgr8";

    const unsigned int frame_count = static_cast<unsigned int>(video.get(CV_CAP_PROP_FRAME_COUNT));

    for (unsigned int i=1; i<=frame_count; ++i)
    {
        video >> cvImage.image; // get a new frame from camera
        // add timestamp
        //cvImage.header.stamp = ros::Time::now();
        image_pub.publish(cvImage.toCompressedImageMsg());
        ROS_INFO_NAMED("Video file streamer", "Frame %d published.", i);

        if(i==frame_count){
            i=1;
            video.set(CAP_PROP_POS_FRAMES, 0);
            ROS_INFO_NAMED("Video file streamer", "Restart video.");
        }

        loop_rate.sleep();
        ros::spinOnce();

        if (!ros::ok())
            break;
    }

    ROS_INFO_NAMED("Video file streamer", "End streaming");

    return 0;
}
