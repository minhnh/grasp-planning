/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */
#include <string>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_planning_node");
    ros::NodeHandle nh("~");
    std::string rgbImgTopic, depthImgTopic;
    nh.getParam("rgb_image_topic", rgbImgTopic);
    nh.getParam("depth_image_topic", depthImgTopic);
    ROS_INFO("will subscribe to image topics: RGB - %s, depth - %s", rgbImgTopic.c_str(), depthImgTopic.c_str());
    ros::spin();
    return 0;
}
