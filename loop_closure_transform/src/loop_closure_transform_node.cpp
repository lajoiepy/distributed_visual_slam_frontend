#include <ros/ros.h>
#include <queue>
#include "kimera-vio/loopclosure/LoopClosureDetector.h"
#include "loop_closure_transform/StereoMatch.h"

class LCDHandler
{
public:
    LCDHandler(VIO::LoopClosureDetectorParams lcd_params):
        lcd_(lcd_params, true)
    {
    };

    void verify_match_images(const loop_closure_transform::StereoMatch::ConstPtr &msg)
    {
        CameraParams cam_param_left_0()
        VIO::StereoFrame stereo_frame_0(    msg->id_0, 
                                            msg->left_0.header.stamp,
                                            VIO::readRosImage(msg->left_0),
                                            CameraParams& cam_param_left,
                                            cv::Mat& right_image,
                                            CameraParams& cam_param_right,
                                            StereoMatchingParams& stereo_matching_params)
        
        FrameId = lcd_.processAndAddFrame(stereo_frame_0);
    }

    void process_next_match() {

        /*lcd_.geometricVerificationNister(
        const FrameId& query_id,
        const FrameId& match_id,
        gtsam::Pose3* camCur_T_camRef_mono);*/
        

        // recoverPose
    }

private:
    VIO::LoopClosureDetector lcd_;
    std::queue<std::pair<VIO::FrameId, VIO::FrameId>> match_queue_;
};



int main(int argc, char** argv) {
    // Initialization
    ros::init( argc, argv, "loop_closure_transform" );
    ros::NodeHandle nh;

    // Geometric transformation engine
    VIO::LoopClosureDetectorParams lcd_params;
    LCDHandler handler(lcd_params);

    // Subscriber
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_match = it.subscribe("verify_match_images", 1000, &LCDHandler::verify_match_images, &handler);

    // Main loop
    while(ros::ok()){
        handler.process_next_match();
        ros::spinOnce();
    }

    // All done
    return 0;
}
