/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */
#ifndef GRASP_PLANNING_IMAGE_DETECTION_GQCNN_NODE_H
#define GRASP_PLANNING_IMAGE_DETECTION_GQCNN_NODE_H

#include <string>
#include <vector>
#include <unordered_set>
#include <chrono>   // NOLINT
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/publisher.h>
#include <message_filters/subscriber.h>
#include <mcr_perception_msgs/DetectImage.h>
#include <mdr_pickup_action/PickupAction.h>
#include <gqcnn/GQCNNGraspPlanner.h>
#include <gqcnn/GQCNNGrasp.h>

namespace mf = message_filters;
namespace sm = sensor_msgs;
namespace it = image_transport;
namespace mpm = mcr_perception_msgs;

struct GraspConfigs
{
public:
    GraspConfigs() : mGripperLinkOffset(0.0f), mMinGraspHeight(0.0f), mMaxGraspReach(0.0f), mMaxWorkspaceDist(0.0f),
                     mMaxGraspNumber(0)
    { }

public:
    std::unordered_set<std::string> mGraspableObjects;
    float mGripperLinkOffset;
    float mMinGraspHeight;
    float mMaxGraspReach;
    float mMaxWorkspaceDist;
    int mMaxGraspNumber;
};

class GraspPlannerGQCNNNode
{
public:
    GraspPlannerGQCNNNode(const ros::NodeHandle&, std::string, std::string, std::string,
                          std::string, std::string, std::string, const GraspConfigs& pConfigs, const std::string&, int);

    ~GraspPlannerGQCNNNode() = default;

    void
    syncCallback(const sm::ImageConstPtr&, const sm::ImageConstPtr&, const sm::CameraInfoConstPtr&);

private:
    void
    triggerCallback(const std_msgs::String::ConstPtr&);

    mpm::DetectImageResponse
    requestDetectionService(const sm::ImageConstPtr&);

    gqcnn::GQCNNGrasp
    requestGqcnnService(const gqcnn::GQCNNGraspPlannerRequest&);

    void
    executeGrasps(const std::vector<gqcnn::GQCNNGrasp>&, const mpm::ImageDetection&, const std::vector<size_t>&,
                  const std_msgs::Header&, double pGraspConfThreshold = sDefaultGraspConfThreshold);

    void
    visualizeGraspsAndObjects(const sm::ImageConstPtr&, const std::vector<gqcnn::GQCNNGrasp> &,
                              const mpm::ImageDetection &, const std::vector<size_t>&, const std::string &,
                              double pGraspConfThreshold = sDefaultGraspConfThreshold);

private:
    const std::string cGraspMarkerTopic = "grasp_markers";
    const std::string cMarkerNamespace = "gqcnn_grasps";
    const std::string cBoxImageTopic = "detection_image";
    static constexpr double sDefaultGraspConfThreshold = 0.01;

private:
    ros::NodeHandle mNodeHandle;
    it::ImageTransport mImageTransport;

    ros::ServiceClient mDetectionClient;
    ros::ServiceClient mGqcnnClient;
    it::Publisher mBoxImagePub;
    ros::Publisher mMarkerPub;
    ros::Publisher mMarkerArrayPub;
    ros::Subscriber mEventSub;
    actionlib::SimpleActionClient<mdr_pickup_action::PickupAction> mPickupClient;
    tf::TransformListener mTfListener;

    // message filters
    typedef mf::sync_policies::ApproximateTime<sm::Image, sm::Image, sm::CameraInfo> ImagePolicy;
    it::SubscriberFilter mRgbImageSub;
    it::SubscriberFilter mDepthImageSub;
    mf::Subscriber<sm::CameraInfo> mCameraInfoSub;
    // Note: synchronizer instance needs to be declared after subscribers to avoid issue described in
    // https://github.com/ros/ros_comm/issues/720
    mf::Synchronizer<ImagePolicy> mSync;

    GraspConfigs mConfigs;
    std::chrono::seconds mWaitTime;
    std::string mTargetFrame;
    bool mTriggered;
};

#endif  // GRASP_PLANNING_IMAGE_DETECTION_GQCNN_NODE_H
