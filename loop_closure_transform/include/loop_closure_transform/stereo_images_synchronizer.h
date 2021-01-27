#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include "loop_closure_transform/stereo_cam_geometric_tools.h"

#ifndef STEREO_IMAGE_SYNCHRONIZER
#define STEREO_IMAGE_SYNCHRONIZER

class StereoImagesSynchronizer {
public:
  StereoImagesSynchronizer(
      const std::string& image_left_topic,
      const std::string& image_right_topic,
      StereoCamGeometricTools& callback_handler
  ) :
    it_(nh_),
    left_image_sub_( it_, image_left_topic, 1 ),
    right_image_sub_( it_, image_right_topic, 1 ),
    sync( MySyncPolicy( 10 ), left_image_sub_, right_image_sub_ )
  {
    sync.registerCallback( boost::bind( &StereoCamGeometricTools::ComputeFeaturesAndDescriptors, &callback_handler, _1, _2 ) );
  }

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  typedef image_transport::SubscriberFilter ImageSubscriber;

  ImageSubscriber left_image_sub_;
  ImageSubscriber right_image_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

  message_filters::Synchronizer< MySyncPolicy > sync;
};

#endif