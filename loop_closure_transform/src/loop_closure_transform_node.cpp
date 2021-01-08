#include <ros/ros.h>
#include <queue>
#include "loop_closure_transform/StereoMatch.h"

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

    // Main loop
    while(ros::ok()){
        //handler.process_next_match();
        ros::spinOnce();
    }

    // All done
    return 0;
}
