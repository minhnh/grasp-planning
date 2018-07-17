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
#include <gqcnn/GQCNNGraspPlanner.h>
#include <gqcnn/BoundingBox.h>
#include <grasp_planning/image_detection_gqcnn_node.h>

ImageDetectionGQCNNNode::ImageDetectionGQCNNNode(const ros::NodeHandle& pNodeHandle, std::string pRgbImgTopic,
                                                 std::string pDepthImgTopic, std::string pCameraInfoTopic,
                                                 std::string pDetectService, std::string pGqcnnService, int pWaitTime)
        : mNodeHandle(pNodeHandle), mImageTransport(mNodeHandle), mRgbImageSub(mImageTransport, pRgbImgTopic, 1),
          mDepthImageSub(mImageTransport, pDepthImgTopic, 1), mCameraInfoSub(mNodeHandle, pCameraInfoTopic, 1),
          mSync(ImagePolicy(10), mRgbImageSub, mDepthImageSub, mCameraInfoSub), mWaitTime(pWaitTime)
{
    ROS_INFO("waiting for image detection service: %s", pDetectService.c_str());
    mDetectionClient = mNodeHandle.serviceClient<mcr_perception_msgs::DetectImage>(pDetectService);
    if (!mDetectionClient.waitForExistence(ros::Duration(pWaitTime)))
    {
        std::stringstream message;
        message << "failed to wait for service: " << pDetectService;
        throw std::runtime_error(message.str());
    }

    ROS_INFO("waiting for GQCNN service: %s", pGqcnnService.c_str());
    mGqcnnClient = mNodeHandle.serviceClient<gqcnn::GQCNNGraspPlanner>(pGqcnnService);
    if (!mGqcnnClient.waitForExistence(ros::Duration(pWaitTime)))
    {
        std::stringstream message;
        message << "failed to wait for service: " << pGqcnnService;
        throw std::runtime_error(message.str());
    }

    ROS_INFO("synchronizing topics: RGB - %s, depth - %s, camera info - %s",
             pRgbImgTopic.c_str(), pDepthImgTopic.c_str(), pCameraInfoTopic.c_str());
    mSync.registerCallback(boost::bind(&ImageDetectionGQCNNNode::syncCallback, this, _1, _2, _3));
}

void
ImageDetectionGQCNNNode::syncCallback(const sm::ImageConstPtr& pRgbImagePtr, const sm::ImageConstPtr& pDepthImagePtr,
                                      const sm::CameraInfoConstPtr& pCamInfoPtr)
{
    std::future<mpm::DetectImageResponse> detectionCall =
            std::async(&ImageDetectionGQCNNNode::requestDetectionService, this, pRgbImagePtr);
    if (detectionCall.wait_for(mWaitTime) == std::future_status::timeout)
    {
        ROS_ERROR("timeout waiting for image detection service call");
        return;
    }

    mpm::DetectImageResponse detectionResult = detectionCall.get();
    if (detectionResult.detections.empty() || detectionResult.detections[0].bounding_boxes.empty())
    {
        ROS_WARN("no detection in image");
        return;
    }

    ROS_INFO("detected %lu boxes in image", detectionResult.detections[0].bounding_boxes.size());
    std::vector<gqcnn::GQCNNGrasp> gqcnnGrasps;
    for (mpm::BoundingBox2D& box : detectionResult.detections[0].bounding_boxes)
    {
        gqcnn::GQCNNGraspPlannerRequest gqcnnRequest;
        gqcnnRequest.camera_info = *pCamInfoPtr;
        gqcnnRequest.color_image = *pRgbImagePtr;
        gqcnnRequest.depth_image = *pDepthImagePtr;
        gqcnnRequest.bounding_box.maxX = box.x_max;
        gqcnnRequest.bounding_box.maxY = box.y_max;
        gqcnnRequest.bounding_box.minX = box.x_min;
        gqcnnRequest.bounding_box.minY = box.y_min;

        std::future<gqcnn::GQCNNGrasp> gqcnnCall =
                std::async(&ImageDetectionGQCNNNode::requestGqcnnService, this, gqcnnRequest);
        if (gqcnnCall.wait_for(mWaitTime) == std::future_status::timeout)
        {
            ROS_ERROR("timeout waiting for GQCNN service call");
            continue;
        }

        gqcnnGrasps.push_back(gqcnnCall.get());
    }

    ROS_INFO("detected %lu grasps", gqcnnGrasps.size());
    visualizeGrasps(gqcnnGrasps);
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

gqcnn::GQCNNGrasp
ImageDetectionGQCNNNode::requestGqcnnService(const gqcnn::GQCNNGraspPlannerRequest& request)
{
    gqcnn::GQCNNGraspPlanner gqcnnSrv;
    gqcnnSrv.request = request;
    if (!mGqcnnClient.call(gqcnnSrv))
    {
        ROS_WARN("failed to call GQCNN service");
    }
    return gqcnnSrv.response.grasp;
}

void
ImageDetectionGQCNNNode::visualizeGrasps(const std::vector<gqcnn::GQCNNGrasp>& grasps)
{
    const gqcnn::GQCNNGrasp* bestGrasp = nullptr;
    double bestSuccessProb = 0.0d;
    for (auto &grasp : grasps)
    {
        if (grasp.grasp_success_prob <= bestSuccessProb)
            continue;

        bestGrasp = &grasp;
        bestSuccessProb = grasp.grasp_success_prob;
    }

    ROS_INFO("best grasp: prob=%.3f, pose.position: %.3f, %.3f, %.3f, .orientation: %.3f, %.3f, %.3f, %.3f",
             bestGrasp->grasp_success_prob, bestGrasp->pose.position.x, bestGrasp->pose.position.y,
             bestGrasp->pose.position.z, bestGrasp->pose.orientation.x, bestGrasp->pose.orientation.y,
             bestGrasp->pose.orientation.z, bestGrasp->pose.orientation.w);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_planning_node");
    ros::NodeHandle nh("~");

    std::string rgbImgTopic, depthImgTopic, detectService, cameraInfoTopic, gqcnnService;
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
    if (!nh.getParam("camera_info_topic", cameraInfoTopic))
    {
        ROS_ERROR("No camera info topic name specified as parameter");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("detection_service_name", detectService))
    {
        ROS_ERROR("No detection service name specified as parameter");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("gqcnn_service_name", gqcnnService))
    {
        ROS_ERROR("No GQCNN service name specified as parameter");
        return EXIT_FAILURE;
    }
    int waitTime;
    nh.param<int>("service_wait_time", waitTime, 10);

    ImageDetectionGQCNNNode graspDetectionNode(nh, rgbImgTopic, depthImgTopic, cameraInfoTopic,
                                               detectService, gqcnnService, waitTime);
    while (ros::ok())
    {
        ros::spin();
    }
    return EXIT_SUCCESS;
}
