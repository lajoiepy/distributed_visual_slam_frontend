#include <string>

#include "ros/ros.h"
#include "loop_closure_transform/ReceiveSeparators.h"
#include "loop_closure_transform/FindMatches.h"
#include "loop_closure_transform/logger.h"

class Communicater
{
private:
    /* data */
    int other_robot_id_;

public:
    bool find_matches_query(loop_closure_transform::FindMatches::Request &req,
                            loop_closure_transform::FindMatches::Response &res);

    bool find_matches_answer(loop_closure_transform::FindMatches::Request &req,
                             loop_closure_transform::FindMatches::Response &res);

    bool found_separators_query(loop_closure_transform::ReceiveSeparators::Request &req,
                               loop_closure_transform::ReceiveSeparators::Response &res);

    bool found_separators_receive(loop_closure_transform::ReceiveSeparators::Request &req,
                                  loop_closure_transform::ReceiveSeparators::Response &res);
    ros::NodeHandle n_;
    Communicater();
};