#include "loop_closure_transform/communication.h"

Communicater::Communicater() 
{
    if (n_.getParam("other_robot_id", other_robot_id_))
    {
        ROS_INFO("Other robot ID is %d ", other_robot_id_);
    }
    else
    {
        ROS_ERROR("Couldn't find other robot ID");
    }
}

bool Communicater::found_loopclosures_query(loop_closure_transform::ReceiveLoopClosures::Request &req,
                                         loop_closure_transform::ReceiveLoopClosures::Response &res)
{
    std::string service_name = "/robot_"+std::to_string(other_robot_id_)+"/found_loopclosures_receive";
    ros::ServiceClient client = n_.serviceClient<loop_closure_transform::ReceiveLoopClosures>(service_name);
    loop_closure_transform::ReceiveLoopClosures srv;
    srv.request = req;
    log_receive_loopclosures_query(req);
    if (client.call(srv))
    {
        log_receive_loopclosures_answer(res);
        res = srv.response;
    }
    else
    {
        ROS_ERROR("Failed to call found_loopclosures_receive service");
        return 1;
    }
    return true;
}

bool Communicater::found_loopclosures_receive(loop_closure_transform::ReceiveLoopClosures::Request &req,
                                            loop_closure_transform::ReceiveLoopClosures::Response &res)
{
    ros::ServiceClient client = n_.serviceClient<loop_closure_transform::ReceiveLoopClosures>("receive_loopclosures_py");
    loop_closure_transform::ReceiveLoopClosures srv;
    srv.request = req;
    if (client.call(srv))
    {
        res = srv.response;
    }
    else
    {
        ROS_ERROR("Failed to call receive_loopclosures_py service");
        return 1;
    }
    return true;
}

bool Communicater::find_matches_query(loop_closure_transform::FindMatches::Request &req,
                                      loop_closure_transform::FindMatches::Response &res)
{
    std::string service_name = "/robot_" + std::to_string(other_robot_id_) + "/find_matches_answer";
    ros::ServiceClient client = n_.serviceClient<loop_closure_transform::FindMatches>(service_name);
    loop_closure_transform::FindMatches srv;
    srv.request = req;
    log_find_matches_query(req);

    if (client.call(srv))
    {

        res = srv.response;
        log_find_matches_answer(res);
    }
    else
    {
        ROS_WARN("Failed to call find_matches_answer service, maybe the robot isn't available");
        return 1;
    }
    return true;
}

bool Communicater::find_matches_answer(loop_closure_transform::FindMatches::Request &req,
                                       loop_closure_transform::FindMatches::Response &res)
{
    ros::ServiceClient client = n_.serviceClient<loop_closure_transform::FindMatches>("find_matches_compute");
    loop_closure_transform::FindMatches srv;
    srv.request = req;
    if (client.call(srv))
    {
        res = srv.response;
    }
    else
    {
        ROS_ERROR("Failed to call find_matches_compute service");
        return 1;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "communication_node");
    Communicater communicater = Communicater();

    ros::ServiceServer service_matches_query = communicater.n_.advertiseService("find_matches_query", &Communicater::find_matches_query, &communicater);
    ros::ServiceServer service_matches_answer = communicater.n_.advertiseService("find_matches_answer", &Communicater::find_matches_answer, &communicater);
    ros::ServiceServer service_sep_query = communicater.n_.advertiseService("found_loopclosures_query", &Communicater::found_loopclosures_query, &communicater);
    ros::ServiceServer service_sep_rec = communicater.n_.advertiseService("found_loopclosures_receive", &Communicater::found_loopclosures_receive, &communicater);

    ros::AsyncSpinner spinner(2); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}