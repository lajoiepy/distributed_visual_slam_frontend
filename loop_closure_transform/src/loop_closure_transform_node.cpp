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

    void verify_match(const loop_closure_transform::StereoMatch::ConstPtr &msg)
    {
        //VIO::StereoFrame stereo_frame_;
        //lcd_.processAndAddFrame(stereo_frame_);
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
    ros::NodeHandle nh_;

    // Geometric transformation engine
    VIO::LoopClosureDetectorParams lcd_params;
    LCDHandler handler(lcd_params);

    // Subscriber
    ros::Subscriber sub_match = nh_.subscribe("verify_match", 1000, &LCDHandler::verify_match, &handler);

    // Main loop
    while(ros::ok()){
        handler.process_next_match();
        ros::spinOnce();
    }

    // All done
    return 0;
}
