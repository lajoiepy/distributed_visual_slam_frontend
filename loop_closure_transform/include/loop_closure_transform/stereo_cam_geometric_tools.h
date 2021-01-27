#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <opencv2/core/core.hpp>

#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/SensorData.h"

#include "loop_closure_transform/EstTransform.h"
#include "loop_closure_transform/EstimatedTransform.h"
#include "loop_closure_transform/StereoExtractResult.h"
#include "loop_closure_transform/MsgConversion.h"
#include "loop_closure_transform/StereoImagePair.h"
#include "loop_closure_transform/Registration.h"
#include "sensor_msgs/CameraInfo.h"

#include <memory>

using namespace rtabmap;
class StereoCamGeometricTools
{
public:
    StereoCamGeometricTools(const sensor_msgs::CameraInfo &camera_info_l, const sensor_msgs::CameraInfo &camera_info_r, const std::string &frame_id, const bool &estimate_stereo_transform_from_tf, const int &nb_min_inliers_loopclosures);


    void ComputeFeaturesAndDescriptors(const loop_closure_transform::StereoImagePair::ConstPtr &msg);
    loop_closure_transform::StereoExtractResult SendFeaturesAndDescriptors();
    bool IsExtractionQueueEmpty(){ return extraction_result_queue_.empty(); };


    void EstimateTransformation(const loop_closure_transform::EstTransform::ConstPtr &msg);
    loop_closure_transform::EstimatedTransform SendEstimatedTransform();
    bool IsTransformQueueEmpty(){ return transform_result_queue_.empty(); };

    
private:
    StereoCameraModel cam_;
    std::unique_ptr<Registration> registration_pipeline_;
    RegistrationInfo info_;
    cv::Size image_size_;

    std::queue<loop_closure_transform::StereoExtractResult> extraction_result_queue_;
    std::queue<loop_closure_transform::EstimatedTransform> transform_result_queue_;
};
