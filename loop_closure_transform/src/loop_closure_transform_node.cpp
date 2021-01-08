#include <ros/ros.h>
#include <kimera-vio/loopclosure/LoopClosureDetector.h>

class LCDHandler
{
public:
    LCDMessageHandler(VIO::LoopClosureDetectorParams lcd_params){
        lcd_ = VIO::LoopClosureDetector(lcd_params, true);
    };

    void verify_match(const StereoMatch::ConstPtr &msg)
    {
    
    }

    void process_next_match() {
        /*LoopClosureDetector::geometricVerificationNister(
        const FrameId& query_id,
        const FrameId& match_id,
        gtsam::Pose3* camCur_T_camRef_mono);*/

        // recoverPose
    }

private:
    VIO::LoopClosureDetector lcd_;
    // Create Queue

};



int main(int argc, char** argv) {
    // Initialization
    ros::init( argc, argv, "loop_closure_transform" );
    ros::NodeHandle nh_;

    // Subscriber
    ros::Subscriber sub_match = nh_.subscribe("verify_match", 1000, &verify_match);

    // Geometric transformation engine
    VIO::LoopClosureDetectorParams lcd_params;
    

    // Main loop
    while(ros::ok()){
        process_next_match();
        ros::spinOnce();
    }

    // All done
    return 0;
}
