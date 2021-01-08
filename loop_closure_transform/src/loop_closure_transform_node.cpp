#include <ros/ros.h>
#include <kimera-vio/loopclosure/LoopClosureDetector.h>

int main(int argc, char** argv) {
   // Intialization
   ros::init( argc, argv, "loop_closure_transform" );
   ros::NodeHandle nh_;

    // Main loop
    while(ros::ok()){
        ros::spinOnce();
    }

   // All done
   return 0;
}
