#include "loop_closure_transform/stereo_cam_geometric_tools.h"

using namespace rtabmap;

StereoCamGeometricTools::StereoCamGeometricTools(const sensor_msgs::CameraInfo &camera_info_l, const sensor_msgs::CameraInfo &camera_info_r, const std::string &frame_id, const bool &estimate_stereo_transform_from_tf, const int &nb_min_inliers_loopclosures)
{
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kInfo);

    tf::TransformListener listener;
    tf::StampedTransform local_transform_tf;

    ros::Time stamp = camera_info_l.header.stamp > camera_info_r.header.stamp ? camera_info_l.header.stamp : camera_info_r.header.stamp;

    try
    {
        listener.waitForTransform("frame_id", camera_info_l.header.frame_id,
                                  stamp, ros::Duration(3.0));

        listener.lookupTransform(frame_id, camera_info_l.header.frame_id,
                                 stamp, local_transform_tf);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    Transform local_transform = transformFromTF(local_transform_tf);

    Transform stereo_transform;
    if (estimate_stereo_transform_from_tf)
    {
        tf::StampedTransform stereo_transform_tf;
        try
        {
            listener.lookupTransform(camera_info_r.header.frame_id, camera_info_l.header.frame_id,
                                     camera_info_l.header.stamp, stereo_transform_tf);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        stereo_transform = transformFromTF(stereo_transform_tf);
    }


    cam_ = stereoCameraModelFromROS(camera_info_l, camera_info_r, local_transform, stereo_transform);

    ParametersMap params;
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpCorrespondenceRatio(), "0.1"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpIterations(), "10"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpMaxCorrespondenceDistance(), "0.3"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpMaxTranslation(), "0"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpPointToPlane(), "true"));
   // params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpVoxelSize(), "0"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpEpsilon(), "0.01"));
    
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisMinInliers(), std::to_string(nb_min_inliers_loopclosures)));

    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisEstimationType(), "1"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisPnPFlags(), "0"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisPnPReprojError(), "2"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisCorFlowWinSize(), "16"));
    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisCorType(), "0"));

    //params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRegStrategy(), "0"));

    registration_pipeline_ = std::unique_ptr<Registration>(Registration::create(params));
}

void StereoCamGeometricTools::ComputeFeaturesAndDescriptors(const sensor_msgs::Image::ConstPtr& image_left, const sensor_msgs::Image::ConstPtr& image_right)
{
    // TODO:: Add Queue
    cv::Mat img_l = cv_bridge::toCvCopy(image_left, sensor_msgs::image_encodings::MONO8)->image;
    cv::Mat img_r = cv_bridge::toCvCopy(image_right, sensor_msgs::image_encodings::MONO8)->image;

    SensorData frame(img_l, img_r, cam_);

    std::vector<cv::KeyPoint> kptsFrom;
    std::vector<cv::Point3f> kptsFrom3D;
    cv::Mat descriptorsFrom;
    RegistrationInfo info1;
    Signature frameSig(frame);
    registration_pipeline_->getFeatures(kptsFrom3D, kptsFrom, descriptorsFrom, frameSig, &info1);

    loop_closure_transform::StereoExtractResult new_result;
    descriptorsToROS(descriptorsFrom, new_result.descriptors);
    keypointsToROS(kptsFrom, new_result.kpts);
    keypoints3DToROS(kptsFrom3D, new_result.kpts3D);
    extraction_result_queue_.push(new_result);
}

loop_closure_transform::StereoExtractResult StereoCamGeometricTools::SendFeaturesAndDescriptors()
{   
    auto result = extraction_result_queue_.front();
    extraction_result_queue_.pop();
    return result;
}

loop_closure_transform::EstimatedTransform StereoCamGeometricTools::SendEstimatedTransform()
{
    auto result = transform_result_queue_.front();
    transform_result_queue_.pop();
    return result;
}

void StereoCamGeometricTools::EstimateTransformation(const loop_closure_transform::EstTransform::ConstPtr &msg)
{
    // TODO: Add queue
    Transform guess(0.0, 0.0, 0.0, 0, 0, 0);

    std::vector<cv::KeyPoint> kptsFrom;
    std::vector<cv::KeyPoint> kptsTo;
    std::vector<cv::Point3f> kptsFrom3D;
    std::vector<cv::Point3f> kptsTo3D;
    cv::Mat descriptorsFrom;
    cv::Mat descriptorsTo;

    kptsFrom3D = keypoints3DFromROS(msg->kptsFrom3D);
    kptsTo3D = keypoints3DFromROS(msg->kptsTo3D);
    kptsFrom = keypointsFromROS(msg->kptsFrom);
    kptsTo = keypointsFromROS(msg->kptsTo);
    descriptorsFrom = descriptorsFromROS(msg->descriptorsFrom);
    descriptorsTo = descriptorsFromROS(msg->descriptorsTo);
    Transform result = registration_pipeline_->computeTransformationFromFeats(
        cam_,
        cam_,
        descriptorsFrom,
        descriptorsTo,
        image_size_,
        kptsFrom3D,
        kptsTo3D,
        kptsFrom,
        kptsTo,
        guess,
        &info_);
    Transform result2 = registration_pipeline_->computeTransformationFromFeats(
        cam_,
        cam_,
        descriptorsFrom,
        descriptorsTo,
        image_size_,
        kptsFrom3D,
        kptsTo3D,
        kptsFrom,
        kptsTo,
        result,
        &info_);

    loop_closure_transform::EstimatedTransform transform;
    transform.key1 = 0; // TODO
    transform.key2 = 0; // TODO
    covToFloat64Msg(info_.covariance, transform.pose.covariance);
    transformToPoseMsg(result2, transform.pose.pose);
    transform_result_queue_.push(transform);
}


/*
int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_cam_geometric_tools_node");
    ros::NodeHandle n;

    bool estimate_stereo_transform_from_tf;
    std::string frame_id;
    int nb_min_inliers_loopclosures;
    if (!n.getParam("estimate_stereo_transform_from_tf", estimate_stereo_transform_from_tf))
    {
        ROS_ERROR("Couldn't find estimate_stereo_transform_from_tf param");
    }

    if (!n.getParam("frame_id", frame_id))
    {
        ROS_ERROR("Couldn't find frame ID");
    }

    if (!n.getParam("loopclosures_min_inliers", nb_min_inliers_loopclosures))
    {
        ROS_ERROR("Couldn't find loopclosures_min_inliers param");
    }

    sensor_msgs::CameraInfoConstPtr camera_info_l_cst_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("left/camera_info", n);
    sensor_msgs::CameraInfoConstPtr camera_info_r_cst_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("right/camera_info", n);
    sensor_msgs::CameraInfo camera_info_l = *camera_info_l_cst_ptr;
    sensor_msgs::CameraInfo camera_info_r = *camera_info_r_cst_ptr;

    StereoCamGeometricTools stereoCamGeometricToolsNode = StereoCamGeometricTools(camera_info_l, camera_info_r, frame_id, estimate_stereo_transform_from_tf, nb_min_inliers_loopclosures);
    ros::ServiceServer service_feats = n.advertiseService("get_features_and_descriptor", &StereoCamGeometricTools::getFeaturesAndDescriptor, &stereoCamGeometricToolsNode);
    ros::ServiceServer service_transf = n.advertiseService("estimate_transformation", &StereoCamGeometricTools::estimateTransformation, &stereoCamGeometricToolsNode);
    ROS_INFO("Stereo camera geometric tools ready");
    ros::spin();

    return 0;
}*/
