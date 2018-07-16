/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */
#include <string>
#include <vector>
#include <future>   // NOLINT
#include <chrono>   // NOLINT
#include <ros/ros.h>
#include <image_transport/subscriber_filter.h>
#include <mcr_perception_msgs/DetectImage.h>
#include <mcr_perception_msgs/BoundingBox2D.h>
#include <grasp_planning/image_detection_gqcnn_node.h>

ImageDetectionGQCNNNode::ImageDetectionGQCNNNode(const ros::NodeHandle& pNodeHandle, std::string pRgbImgTopic,
                                                 std::string pDepthImgTopic, std::string pDetectService,
                                                 int pWaitTime)
        : mNodeHandle(pNodeHandle), mImageTransport(mNodeHandle),
          mRgbImageSub(mImageTransport, pRgbImgTopic, 1), mDepthImageSub(mImageTransport, pDepthImgTopic, 1),
          mSync(TwoImagePolicy(10), mRgbImageSub, mDepthImageSub), mWaitTime(pWaitTime)
{
    ROS_INFO("waiting for image detection service: %s", pDetectService.c_str());
    mDetectionClient = mNodeHandle.serviceClient<mcr_perception_msgs::DetectImage>(pDetectService);
    if (!mDetectionClient.waitForExistence(ros::Duration(pWaitTime)))
    {
        std::stringstream message;
        message << "failed to wait for service: " << pDetectService;
        throw std::runtime_error(message.str());
    }

    ROS_INFO("synchronizing image topics: RGB - %s, depth - %s", pRgbImgTopic.c_str(), pDepthImgTopic.c_str());
    mSync.registerCallback(boost::bind(&ImageDetectionGQCNNNode::syncCallback, this, _1, _2));
}

void
ImageDetectionGQCNNNode::syncCallback(const sm::ImageConstPtr& pRgbImagePtr, const sm::ImageConstPtr& pDepthImagePtr)
{
    std::future<mpm::DetectImageResponse> detectionCall =
            std::async(&ImageDetectionGQCNNNode::requestDetectionService, this, pRgbImagePtr);
    if (detectionCall.wait_for(mWaitTime) == std::future_status::timeout)
    {
        ROS_WARN("timeout waiting for image detection service call");
        return;
    }

    mpm::DetectImageResponse detectionResult = detectionCall.get();
    if (detectionResult.detections.empty() || detectionResult.detections[0].bounding_boxes.empty())
    {
        return;
    }

    mpm::BoundingBox2D firstBox = detectionResult.detections[0].bounding_boxes[0];
    ROS_INFO("first box: x_min = %f, y_min = %f, x_max = %f, y_max = %f",
             firstBox.x_min, firstBox.y_min, firstBox.x_max, firstBox.y_max);
}

mpm::DetectImageResponse
ImageDetectionGQCNNNode::requestDetectionService(const sm::ImageConstPtr& pImagePtr)
{
    mpm::DetectImage detectSrv;
    detectSrv.request.images.push_back(*pImagePtr);
    if (!mDetectionClient.call(detectSrv))
    {
        ROS_WARN("failed to call image detection service");
    }

    return detectSrv.response;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_planning_node");
    ros::NodeHandle nh("~");

    std::string rgbImgTopic, depthImgTopic, detectService;
    if (!nh.getParam("rgb_image_topic", rgbImgTopic))
    {
        ROS_ERROR("No RGB image topic specified as parameter");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("depth_image_topic", depthImgTopic))
    {
        ROS_ERROR("No depth image topic specified as parameter");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("detection_service_name", detectService))
    {
        ROS_ERROR("No detection service name specified as parameter");
        return EXIT_FAILURE;
    }
    int waitTime;
    nh.param<int>("service_wait_time", waitTime, 10);

    ImageDetectionGQCNNNode graspDetectionNode(nh, rgbImgTopic, depthImgTopic, detectService, waitTime);
    while (ros::ok())
    {
        ros::spin();
    }
    return EXIT_SUCCESS;
}
