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
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <mcr_perception_msgs/DetectImage.h>

namespace mf = message_filters;
namespace sm = sensor_msgs;
namespace ip = image_transport;
namespace mpm = mcr_perception_msgs;

typedef mf::sync_policies::ApproximateTime<sm::Image, sm::Image> TwoImagePolicy;

class ImageDetectionGQCNNNode
{
public:
    ImageDetectionGQCNNNode(const ros::NodeHandle&, std::string, std::string, std::string, int);

    ~ImageDetectionGQCNNNode() { }

    void
    syncCallback(const sm::ImageConstPtr&, const sm::ImageConstPtr&);

    mpm::DetectImageResponse
    requestDetectionService(const sm::ImageConstPtr&);

private:
    ros::NodeHandle mNodeHandle;
    ip::ImageTransport mImageTransport;
    ip::SubscriberFilter mRgbImageSub;
    ip::SubscriberFilter mDepthImageSub;
    // Note: synchronizer instance needs to be declared after subscribers to avoid issue described in
    // https://github.com/ros/ros_comm/issues/720
    mf::Synchronizer<TwoImagePolicy> mSync;
    ros::ServiceClient mDetectionClient;
    std::chrono::seconds mWaitTime;
};

#endif  // GRASP_PLANNING_IMAGE_DETECTION_GQCNN_NODE_H
