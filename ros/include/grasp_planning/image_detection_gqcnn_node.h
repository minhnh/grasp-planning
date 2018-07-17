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
#include <chrono>   // NOLINT
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <mcr_perception_msgs/DetectImage.h>
#include <gqcnn/GQCNNGraspPlanner.h>
#include <gqcnn/GQCNNGrasp.h>

namespace mf = message_filters;
namespace sm = sensor_msgs;
namespace ip = image_transport;
namespace mpm = mcr_perception_msgs;

class ImageDetectionGQCNNNode
{
public:
    ImageDetectionGQCNNNode(const ros::NodeHandle&, std::string, std::string, std::string,
                            std::string, std::string, int);

    ~ImageDetectionGQCNNNode() { }

    void
    syncCallback(const sm::ImageConstPtr&, const sm::ImageConstPtr&, const sm::CameraInfoConstPtr&);

private:
    mpm::DetectImageResponse
    requestDetectionService(const sm::ImageConstPtr&);
    gqcnn::GQCNNGrasp
    requestGqcnnService(const gqcnn::GQCNNGraspPlannerRequest&);
    void
    visualizeGrasps(const std::vector<gqcnn::GQCNNGrasp>&);

private:
    ros::NodeHandle mNodeHandle;
    ip::ImageTransport mImageTransport;

    // message filters
    typedef mf::sync_policies::ApproximateTime<sm::Image, sm::Image, sm::CameraInfo> ImagePolicy;
    ip::SubscriberFilter mRgbImageSub;
    ip::SubscriberFilter mDepthImageSub;
    mf::Subscriber<sm::CameraInfo> mCameraInfoSub;
    // Note: synchronizer instance needs to be declared after subscribers to avoid issue described in
    // https://github.com/ros/ros_comm/issues/720
    mf::Synchronizer<ImagePolicy> mSync;

    ros::ServiceClient mDetectionClient;
    ros::ServiceClient mGqcnnClient;

    std::chrono::seconds mWaitTime;
};

#endif  // GRASP_PLANNING_IMAGE_DETECTION_GQCNN_NODE_H
