#include <ros/ros.h>
#include <queue>
#include "loop_closure_transform/StereoMatch.h"
#include "loop_closure_transform/stereoCamGeometricTools.h"

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

    StereoCamGeometricTools stereoCamGeometricToolsNode = StereoCamGeometricTools(camera_info_l, camera_info_r, frame_id, estimate_stereo_transform_from_tf, nb_min_inliers_loopclosures);
    //ros::ServiceServer service_feats = nh.advertiseService("get_features_and_descriptor", &StereoCamGeometricTools::getFeaturesAndDescriptor, &stereoCamGeometricToolsNode);
    //ros::ServiceServer service_transf = nh.advertiseService("estimate_transformation", &StereoCamGeometricTools::estimateTransformation, &stereoCamGeometricToolsNode);
    ros::Subscriber sub_features_descriptors = nh.subscribe("features_and_descriptors", 10, &StereoCamGeometricTools::getFeaturesAndDescriptor, &stereoCamGeometricToolsNode);
    ros::Subscriber sub_transform = nh.subscribe("estimate_transform", 10, &StereoCamGeometricTools::estimateTransformation, &stereoCamGeometricToolsNode);
    
    ROS_INFO("Stereo camera geometric tools ready");


    // Main loop
    while(ros::ok()){
        //handler.process_next_match();
        ros::spinOnce();
    }

    // All done
    return 0;
}
