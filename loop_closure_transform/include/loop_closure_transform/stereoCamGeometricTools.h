#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "loop_closure_transform/EstTransform.h"
#include "loop_closure_transform/MsgConversion.h"
#include "loop_closure_transform/GetFeatsAndDesc.h"

#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/SensorData.h"
#include <opencv2/core/core.hpp>
#include "loop_closure_transform/myRegistration.h"

#include "sensor_msgs/CameraInfo.h"
using namespace rtabmap;
class StereoCamGeometricTools
{
private:
    StereoCameraModel cam_;
    Registration *registrationPipeline_;
    RegistrationInfo info_;
    cv::Size image_size_;

public:
    bool
    getFeaturesAndDescriptor(loop_closure_transform::GetFeatsAndDesc::Request &req,
                             loop_closure_transform::GetFeatsAndDesc::Response &res);
    bool estimateTransformation(loop_closure_transform::EstTransform::Request &req,
                                loop_closure_transform::EstTransform::Response &res);
    StereoCamGeometricTools(const sensor_msgs::CameraInfo &camera_info_l, const sensor_msgs::CameraInfo &camera_info_r, const std::string &frame_id, const bool &estimate_stereo_transform_from_tf, const int &nb_min_inliers_separators);
    // void ~StereoCamGeometricTools();
};