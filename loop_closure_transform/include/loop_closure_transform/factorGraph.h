#include "ros/ros.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <rtabmap_ros/OdomInfo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/dataset.h>

#include "loop_closure_transform/MsgConversion.h"
#include "loop_closure_transform/ReceiveLoopClosures.h"
typedef struct PoseWithCovariance PoseWithCovariance;

struct PoseWithCovariance
{
    gtsam::Pose3 pose;
    gtsam::Matrix covariance_matrix;
};

class FactorGraphData
{
private:
    /* data */
    gtsam::NonlinearFactorGraph pose_graph_;
    gtsam::Values poses_initial_guess_;
    gtsam::Pose3 cur_pose_;
    int local_robot_id_;
    int other_robot_id_;
    unsigned char other_robot_id_char_;
    unsigned char local_robot_id_char_;
    int nb_keyframes_;
    PoseWithCovariance accumulated_transform_;
    void poseCompose(const PoseWithCovariance &a,
                     const PoseWithCovariance &b,
                     PoseWithCovariance &out);
    bool set_fixed_covariance_;
    float translation_std_;
    float rotation_std_;

public:
    void
    addOdometry(const rtabmap_ros::OdomInfo::ConstPtr &msg);
    bool addloopclosures(loop_closure_transform::ReceiveLoopClosures::Request &req,
                       loop_closure_transform::ReceiveLoopClosures::Response &res);
    void manuallySetCovMat(gtsam::Matrix &cov_mat_to_replace);
    FactorGraphData(ros::NodeHandle n);
    ~FactorGraphData();
};

void resetPoseWithCovariance(PoseWithCovariance &toReset);