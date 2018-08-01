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
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <tf/transform_listener.h>
#include <image_transport/subscriber_filter.h>
#include <visualization_msgs/MarkerArray.h>
#include <mcr_perception_msgs/DetectImage.h>
#include <mcr_perception_msgs/BoundingBox2D.h>
#include <mas_perception_libs/image_bounding_box.h>
#include <mdr_pickup_action/PickupAction.h>
#include <gqcnn/GQCNNGraspPlanner.h>
#include <gqcnn/BoundingBox.h>
#include <grasp_planning/image_detection_gqcnn_node.h>

namespace mpl = mas_perception_libs;

const std::string ImageDetectionGQCNNNode::cAllowedLabels[] = { "bottle", "cup", "remote", "vase", "cake", "ball" };

ImageDetectionGQCNNNode::ImageDetectionGQCNNNode(const ros::NodeHandle& pNodeHandle, std::string pRgbImgTopic,
                                                 std::string pDepthImgTopic, std::string pCameraInfoTopic,
                                                 std::string pDetectService, std::string pGqcnnService,
                                                 std::string pPickupActionServer, const std::string& pTargetFrame,
                                                 int pWaitTime)
        : mNodeHandle(pNodeHandle), mImageTransport(mNodeHandle), mRgbImageSub(mImageTransport, pRgbImgTopic, 1),
          mDepthImageSub(mImageTransport, pDepthImgTopic, 1), mCameraInfoSub(mNodeHandle, pCameraInfoTopic, 1),
          mSync(ImagePolicy(5), mRgbImageSub, mDepthImageSub, mCameraInfoSub), mPickupClient(pPickupActionServer),
          mTargetFrame(pTargetFrame), mWaitTime(pWaitTime), mTriggered(false)
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

    ROS_INFO("waiting for pickup action server: %s", pPickupActionServer.c_str());
    if (!mPickupClient.waitForServer(ros::Duration(pWaitTime)))
    {
        std::stringstream message;
        message << "failed to wait for pickup action server: " << pPickupActionServer;
        throw std::runtime_error(message.str());
    }

    mMarkerPub = mNodeHandle.advertise<visualization_msgs::Marker>(cGraspMarkerTopic, 1);
    mMarkerArrayPub = mNodeHandle.advertise<visualization_msgs::MarkerArray>(cGraspMarkerTopic + "_array", 1);
    mBoxImagePub = mImageTransport.advertise(cBoxImageTopic, 1);
    mEventSub = mNodeHandle.subscribe("event_in", 1, &ImageDetectionGQCNNNode::triggerCallback, this);

    ROS_INFO("synchronizing topics: RGB - %s, depth - %s, camera info - %s",
             pRgbImgTopic.c_str(), pDepthImgTopic.c_str(), pCameraInfoTopic.c_str());
    mSync.registerCallback(boost::bind(&ImageDetectionGQCNNNode::syncCallback, this, _1, _2, _3));
}

void
ImageDetectionGQCNNNode::triggerCallback(const std_msgs::String::ConstPtr& msg)
{
    mTriggered = true;
}

void
ImageDetectionGQCNNNode::syncCallback(const sm::ImageConstPtr& pRgbImagePtr, const sm::ImageConstPtr& pDepthImagePtr,
                                      const sm::CameraInfoConstPtr& pCamInfoPtr)
{
    if (!mTriggered)
    {
        ROS_INFO("not triggered");
        ros::Duration(0.5).sleep();
        return;
    }
    mTriggered = false;

    ROS_INFO("calling detection service");
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
        bool graspable = false;
        for (auto & allowedLabel : cAllowedLabels)
        {
            if (detectionResult.detections[0].classes[i].find(allowedLabel) != std::string::npos)
            {
                graspable = true;
            }
        }
        if (!graspable)
        {
            ROS_INFO("skipping ungraspable object: %s", detectionResult.detections[0].classes[i].c_str());
            continue;
        }

        ROS_INFO("planning grasp for: %s (confidence %.3f)", detectionResult.detections[0].classes[i].c_str(),
                 detectionResult.detections[0].probabilities[i]);
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
    ROS_INFO("finished planning %lu grasps, executing grasps...", selectedBoxIndices.size());
    executeGrasps(gqcnnGrasps, detectionResult.detections[0], selectedBoxIndices, pCamInfoPtr->header);

    ROS_INFO("visualizing grasp and objects");
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
ImageDetectionGQCNNNode::executeGrasps(const std::vector<gqcnn::GQCNNGrasp> &pGrasps,
                                       const mpm::ImageDetection &pDetection, const std::vector<size_t>& pBoxIndices,
                                       const std_msgs::Header& pHeader, double pGraspConfThreshold)
{
    try
    {
        mTfListener.waitForTransform(mTargetFrame, pHeader.frame_id,
                                     pHeader.stamp, ros::Duration(1.0));
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("failed to wait for transform from frame '%s' to frame '%s': %s",
                  pHeader.frame_id.c_str(), mTargetFrame.c_str(), ex.what());
        return;
    }

    for (size_t i = 0; i < pBoxIndices.size(); i++)
    {
        if (pGrasps[i].grasp_success_prob < pGraspConfThreshold)
        {
            ROS_WARN("grasp for '%s' has low confidence threshold (%.3f)",
                     pDetection.classes[pBoxIndices[i]].c_str(), pGrasps[i].grasp_success_prob);
        }

        geometry_msgs::PoseStamped objPose, transformedPose;
        objPose.header = pHeader;
        objPose.pose = pGrasps[i].pose;
        try
        {
            ros::Time commonTime;
            mTfListener.getLatestCommonTime(mTargetFrame, objPose.header.frame_id, commonTime, NULL);
            objPose.header.stamp = commonTime;
            mTfListener.transformPose(mTargetFrame, objPose, transformedPose);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("unable to transform pose of object '%s' from frame '%s' to frame '%s': %s",
                      pDetection.classes[pBoxIndices[i]].c_str(), pHeader.frame_id.c_str(), mTargetFrame.c_str(),
                      ex.what());
            continue;
        }

        ROS_INFO("label %s: coord mean (frame {%s}): x=%.3f, y=%.3f, z=%.3f",
                 pDetection.classes[pBoxIndices[i]].c_str(), transformedPose.header.frame_id.c_str(),
                 transformedPose.pose.position.x, transformedPose.pose.position.y, transformedPose.pose.position.z);

        if (transformedPose.pose.position.x > 0.9)
        {
            ROS_INFO("skipping far away object");
            continue;
        }

        transformedPose.pose.position.x -= cGripperLinkOffset;
        if (transformedPose.pose.position.z < cTableHeight)
        {
            transformedPose.pose.position.z = cTableHeight;
            ROS_INFO("readjusting height of object to %.2f", transformedPose.pose.position.z);
        }

        // send actionlib goal
        mdr_pickup_action::PickupGoal goal;
        goal.pose = transformedPose;
        goal.closed_gripper_joint_values.push_back(-0.3f);
        mPickupClient.sendGoal(goal);
        if (!mPickupClient.waitForResult(ros::Duration(mWaitTime.count())))
        {
            ROS_ERROR("failed to wait for pickup action result");
            continue;
        }

        auto result = mPickupClient.getState();
        ROS_INFO("pickup action finished: %s, stop executing grasps", result.toString().c_str());
        break;
    }
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

    std::string rgbImgTopic, depthImgTopic, detectService, cameraInfoTopic, gqcnnService,
                pickupActionServer, target_frame;
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
    if (!nh.getParam("pickup_action_server", pickupActionServer))
    {
        ROS_ERROR("No pickup action server name specified as parameter");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("gqcnn_service_name", gqcnnService))
    {
        ROS_ERROR("No GQCNN service name specified as parameter");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("target_frame", target_frame))
    {
        ROS_ERROR("No target frame for grasping specified as parameter");
        return EXIT_FAILURE;
    }
    int waitTime;
    nh.param<int>("service_wait_time", waitTime, 10);

    ImageDetectionGQCNNNode graspDetectionNode(nh, rgbImgTopic, depthImgTopic, cameraInfoTopic, detectService,
                                               gqcnnService, pickupActionServer, target_frame, waitTime);
    while (ros::ok())
    {
        ros::spin();
    }
    return EXIT_SUCCESS;
}
