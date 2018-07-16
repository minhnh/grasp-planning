/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */
#include <string>
#include <ros/ros.h>
#include <image_transport/subscriber_filter.h>
#include <grasp_planning/image_detection_gqcnn_node.h>

ImageDetectionGQCNNNode::ImageDetectionGQCNNNode(
        const ros::NodeHandle& pNodeHandle, std::string pRgbImgTopic, std::string pDepthImgTopic)
        : mNodeHandle(pNodeHandle), mImageTransport(mNodeHandle),
          mRgbImageSub(mImageTransport, pRgbImgTopic, 1), mDepthImageSub(mImageTransport, pDepthImgTopic, 1),
          mSync(TwoImagePolicy(10), mRgbImageSub, mDepthImageSub)
{
    mSync.registerCallback(boost::bind(&ImageDetectionGQCNNNode::syncCallback, this, _1, _2));
}

void
ImageDetectionGQCNNNode::syncCallback(const sm::ImageConstPtr& pRgbImagePtr, const sm::ImageConstPtr& pDepthImagePtr)
{
    ROS_INFO("received synchronized images");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_planning_node");
    ros::NodeHandle nh("~");

    std::string rgbImgTopic, depthImgTopic;
    nh.getParam("rgb_image_topic", rgbImgTopic);
    nh.getParam("depth_image_topic", depthImgTopic);
    ROS_INFO("synchronizing image topics: RGB - %s, depth - %s", rgbImgTopic.c_str(), depthImgTopic.c_str());

    ImageDetectionGQCNNNode graspDetectionNode(nh, rgbImgTopic, depthImgTopic);
    while (ros::ok())
    {
        ros::spin();
    }
    return EXIT_SUCCESS;
}
