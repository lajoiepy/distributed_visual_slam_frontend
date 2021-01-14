#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "loop_closure_transform/EstTransform.h"
#include "loop_closure_transform/MsgConversion.h"
#include "loop_closure_transform/StereoImagePair.h"

#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/SensorData.h"
#include <opencv2/core/core.hpp>
#include "loop_closure_transform/Registration.h"

#include "sensor_msgs/CameraInfo.h"
using namespace rtabmap;
class StereoCamGeometricTools
{
public:
    void getFeaturesAndDescriptor(const loop_closure_transform::StereoImagePair::ConstPtr &msg);
    void estimateTransformation(const loop_closure_transform::EstTransform::ConstPtr &msg);
    StereoCamGeometricTools(const sensor_msgs::CameraInfo &camera_info_l, const sensor_msgs::CameraInfo &camera_info_r, const std::string &frame_id, const bool &estimate_stereo_transform_from_tf, const int &nb_min_inliers_loopclosures);
    // void ~StereoCamGeometricTools();

private:
    StereoCameraModel cam_;
    Registration *registrationPipeline_;
    RegistrationInfo info_;
    cv::Size image_size_;

};
