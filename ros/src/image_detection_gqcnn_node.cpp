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
#include <visualization_msgs/MarkerArray.h>
#include <mcr_perception_msgs/DetectImage.h>
#include <mcr_perception_msgs/BoundingBox2D.h>
#include <mas_perception_libs/image_bounding_box.h>
#include <gqcnn/GQCNNGraspPlanner.h>
#include <gqcnn/BoundingBox.h>
#include <grasp_planning/image_detection_gqcnn_node.h>

namespace mpl = mas_perception_libs;

ImageDetectionGQCNNNode::ImageDetectionGQCNNNode(const ros::NodeHandle& pNodeHandle, std::string pRgbImgTopic,
                                                 std::string pDepthImgTopic, std::string pCameraInfoTopic,
                                                 std::string pDetectService, std::string pGqcnnService, int pWaitTime)
        : mNodeHandle(pNodeHandle), mImageTransport(mNodeHandle), mRgbImageSub(mImageTransport, pRgbImgTopic, 1),
          mDepthImageSub(mImageTransport, pDepthImgTopic, 1), mCameraInfoSub(mNodeHandle, pCameraInfoTopic, 1),
          mSync(ImagePolicy(1), mRgbImageSub, mDepthImageSub, mCameraInfoSub), mWaitTime(pWaitTime)
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

    mMarkerPub = mNodeHandle.advertise<visualization_msgs::Marker>(cGraspMarkerTopic, 1);
    mMarkerArrayPub = mNodeHandle.advertise<visualization_msgs::MarkerArray>(cGraspMarkerTopic + "_array", 1);
    mBoxImagePub = mImageTransport.advertise(cBoxImageTopic, 1);

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
    std::vector<size_t> selectedBoxIndices;
    for (size_t i = 0; i < detectionResult.detections[0].classes.size(); i++)
    {
        // skipping tables TODO(minhnh): limit to a list of graspable objects
        if (detectionResult.detections[0].classes[i].find("table") != std::string::npos)
        {
            continue;
        }

        gqcnn::GQCNNGraspPlannerRequest gqcnnRequest;
        gqcnnRequest.camera_info = *pCamInfoPtr;
        gqcnnRequest.color_image = *pRgbImagePtr;
        gqcnnRequest.depth_image = *pDepthImagePtr;
        gqcnnRequest.bounding_box.maxX = detectionResult.detections[0].bounding_boxes[i].x_max;
        gqcnnRequest.bounding_box.maxY = detectionResult.detections[0].bounding_boxes[i].y_max;
        gqcnnRequest.bounding_box.minX = detectionResult.detections[0].bounding_boxes[i].x_min;
        gqcnnRequest.bounding_box.minY = detectionResult.detections[0].bounding_boxes[i].y_min;

        std::future<gqcnn::GQCNNGrasp> gqcnnCall =
                std::async(&ImageDetectionGQCNNNode::requestGqcnnService, this, gqcnnRequest);
        if (gqcnnCall.wait_for(mWaitTime) == std::future_status::timeout)
        {
            ROS_ERROR("timeout waiting for GQCNN service call");
            continue;
        }

        selectedBoxIndices.push_back(i);
        gqcnnGrasps.push_back(gqcnnCall.get());
    }
    ROS_INFO("finished planning %lu grasps", selectedBoxIndices.size());

    visualizeGraspsAndObjects(pRgbImagePtr, gqcnnGrasps, detectionResult.detections[0], selectedBoxIndices,
                              pCamInfoPtr->header.frame_id);
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
ImageDetectionGQCNNNode::visualizeGraspsAndObjects(const sm::ImageConstPtr& pImagePtr,
                                                   const std::vector<gqcnn::GQCNNGrasp> &pGrasps,
                                                   const mpm::ImageDetection &pDetection,
                                                   const std::vector<size_t>& pBoxIndices,
                                                   const std::string &pFrameId, double pGraspConfThreshold)
{
    std::vector<mpl::BoundingBox2D> boxes = mpl::imageDetectionToBoundingBoxVect(pDetection);
    sensor_msgs::ImagePtr drawnImage = mpl::drawLabeledBoxesImgMsg(*pImagePtr, boxes);
    mBoxImagePub.publish(drawnImage);

    if (mMarkerArrayPub.getNumSubscribers() == 0)
    {
        ROS_INFO("skipping markers publication since there is no subscriber");
        return;
    }

    if (pBoxIndices.size() != pGrasps.size())
    {
        ROS_ERROR("number of GQCNN grasp messages differs from number of selected objects");
        return;
    }

    // remove all previous markers
    visualization_msgs::Marker delMarker;
    delMarker.header.frame_id = pFrameId;
    delMarker.header.stamp = ros::Time::now();
    delMarker.action = visualization_msgs::Marker::DELETEALL;
    delMarker.ns = cMarkerNamespace;
    mMarkerPub.publish(delMarker);

    visualization_msgs::MarkerArray markerArray;
    std_msgs::ColorRGBA defaultColor;
    defaultColor.a = 1.0f;
    defaultColor.b = 1.0f;
    int id = 0;
    for (size_t i = 0; i < pBoxIndices.size(); i++)
    {
        if (pGrasps[i].grasp_success_prob < pGraspConfThreshold)
            continue;

        visualization_msgs::Marker graspMarker;
        graspMarker.ns = cMarkerNamespace;
        graspMarker.id = id;
        graspMarker.header.frame_id = pFrameId;
        graspMarker.header.stamp = ros::Time::now();
        graspMarker.type = visualization_msgs::Marker::ARROW;
        graspMarker.action = visualization_msgs::Marker::ADD;
        graspMarker.scale.x = 0.1;     // length (10 cm)
        graspMarker.scale.y = 0.01;    // width
        graspMarker.scale.z = 0.01;    // height
        graspMarker.color = defaultColor;
        graspMarker.pose = pGrasps[i].pose;
        graspMarker.lifetime = ros::Duration(20.0);
        markerArray.markers.push_back(graspMarker);
        id++;

        visualization_msgs::Marker objectLblMarker;
        objectLblMarker.ns = cMarkerNamespace;
        objectLblMarker.id = id;
        objectLblMarker.header.frame_id = pFrameId;
        objectLblMarker.header.stamp = ros::Time::now();
        objectLblMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        objectLblMarker.action = visualization_msgs::Marker::ADD;
        objectLblMarker.scale.z = 0.05;    // height of letter 'A'
        objectLblMarker.color = defaultColor;
        std::stringstream label;
        label << pDetection.classes[pBoxIndices[i]] << " (" << pGrasps[i].grasp_success_prob << ")";
        objectLblMarker.text = label.str();
        objectLblMarker.pose = pGrasps[i].pose;
        objectLblMarker.lifetime = ros::Duration(20.0);
        markerArray.markers.push_back(objectLblMarker);
        id++;
    }

    mMarkerArrayPub.publish(markerArray);
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
