#include <ros/ros.h>
#include <queue>
#include "loop_closure_transform/StereoMatch.h"
#include "loop_closure_transform/stereo_cam_geometric_tools.h"

class LCDHandler
{
public:
    LCDHandler()
    {
    };

    void verify_match_images(const loop_closure_transform::StereoMatch::ConstPtr &msg)
    {
       
    }

    void process_next_match() {

    }

private:
    // Add queue
};



int main(int argc, char** argv) {
    // Initialization
    ros::init( argc, argv, "loop_closure_transform" );
    ros::NodeHandle nh;

    // Geometric transformation engine
    LCDHandler handler();

    // Subscriber
    //image_transport::ImageTransport it(nh);
    //image_transport::Subscriber sub_match = it.subscribe("verify_match_images", 1000, &LCDHandler::verify_match_images, &handler);

    //StereoCamGeometricTools stereo_tools;
    bool estimate_stereo_transform_from_tf = true;
    std::string frame_id = "base_line";
    int nb_min_inliers_loopclosures = 20;
    /*if (!n.getParam("estimate_stereo_transform_from_tf", estimate_stereo_transform_from_tf))
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
    }*/

    sensor_msgs::CameraInfoConstPtr camera_info_l_cst_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("left/camera_info", nh);
    sensor_msgs::CameraInfoConstPtr camera_info_r_cst_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("right/camera_info", nh);
    sensor_msgs::CameraInfo camera_info_l = *camera_info_l_cst_ptr;
    sensor_msgs::CameraInfo camera_info_r = *camera_info_r_cst_ptr;

    StereoCamGeometricTools stereo_cam_geometric_tools = StereoCamGeometricTools(camera_info_l, camera_info_r, frame_id, estimate_stereo_transform_from_tf, nb_min_inliers_loopclosures);
    //ros::ServiceServer service_feats = nh.advertiseService("get_features_and_descriptor", &StereoCamGeometricTools::getFeaturesAndDescriptor, &stereo_cam_geometric_tools);
    //ros::ServiceServer service_transf = nh.advertiseService("estimate_transformation", &StereoCamGeometricTools::estimateTransformation, &stereo_cam_geometric_tools);
    ros::Subscriber sub_features_descriptors = nh.subscribe("compute_features_and_descriptors", 10, &StereoCamGeometricTools::ComputeFeaturesAndDescriptors, &stereo_cam_geometric_tools);
    ros::Publisher pub_features_descriptors = nh.advertise<loop_closure_transform::StereoExtractResult >("result_features_and_descriptors", 1000);
    ros::Subscriber sub_transform = nh.subscribe("estimate_transform", 10, &StereoCamGeometricTools::EstimateTransformation, &stereo_cam_geometric_tools);
    ros::Publisher pub_transform = nh.advertise<loop_closure_transform::EstimatedTransform>("result_transform", 1000);

    ROS_INFO("Stereo camera geometric tools ready");


    // Main loop
    while(ros::ok()){
        
        if (!stereo_cam_geometric_tools.IsExtractionQueueEmpty()) {
            pub_features_descriptors.publish(stereo_cam_geometric_tools.SendFeaturesAndDescriptors());
        }

        if (!stereo_cam_geometric_tools.IsTransformQueueEmpty()) {
            pub_features_descriptors.publish(stereo_cam_geometric_tools.SendEstimatedTransform());
        }

        ros::spinOnce();
    }

    // All done
    return 0;
}
