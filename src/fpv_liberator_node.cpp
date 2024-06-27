#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "fpv_liberator.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "fpv_liberator_node");
    ros::NodeHandle nh;

    FPVLiberator fpv;
    if (!fpv.initialize()) {
        ROS_ERROR("Failed to initialize FPV Liberator");
        return 1;
    }

    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("fpv_image", 1);
    
    fpv.run();

    ros::Rate loop_rate(30);  // Adjust as needed
    while (ros::ok()) {
        cv::Mat frame = fpv.getLatestFrame();
        if (!frame.empty()) {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            image_pub.publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    fpv.stop();
    return 0;
}