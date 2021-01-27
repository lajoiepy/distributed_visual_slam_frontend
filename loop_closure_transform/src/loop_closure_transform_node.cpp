#include <ros/ros.h>
#include <queue>
#include "loop_closure_transform/StereoMatch.h"
#include "loop_closure_transform/stereo_images_synchronizer.h"

int main(int argc, char** argv) {
    // Initialization
    ros::init( argc, argv, "loop_closure_transform" );
    ros::NodeHandle nh;

    std::string frame_id;
    if (!nh.getParam("frame_id", frame_id))
    {
        ROS_ERROR("Couldn't find frame ID");
    }

    bool estimate_stereo_transform_from_tf = true;
    int nb_min_inliers_loopclosures = 20;

    sensor_msgs::CameraInfoConstPtr camera_info_l_cst_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("left/camera_info", nh);
    sensor_msgs::CameraInfoConstPtr camera_info_r_cst_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("right/camera_info", nh);
    sensor_msgs::CameraInfo camera_info_l = *camera_info_l_cst_ptr;
    sensor_msgs::CameraInfo camera_info_r = *camera_info_r_cst_ptr;

    StereoCamGeometricTools stereo_cam_geometric_tools = StereoCamGeometricTools(camera_info_l, camera_info_r, frame_id, estimate_stereo_transform_from_tf, nb_min_inliers_loopclosures);
    //ros::Subscriber sub_features_descriptors = nh.subscribe("compute_features_and_descriptors", 10, &StereoCamGeometricTools::ComputeFeaturesAndDescriptors, &stereo_cam_geometric_tools);
    ros::Publisher pub_features_descriptors = nh.advertise<loop_closure_transform::StereoExtractResult >("result_features_and_descriptors", 1000);
    ros::Subscriber sub_transform = nh.subscribe("estimate_transform", 10, &StereoCamGeometricTools::EstimateTransformation, &stereo_cam_geometric_tools);
    ros::Publisher pub_transform = nh.advertise<loop_closure_transform::EstimatedTransform>("result_transform", 1000);

    StereoImagesSynchronizer stereo_sync(
      "left/image_rect",
      "right/image_rect",
      stereo_cam_geometric_tools
    );

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
