/*
 * Copyright (c) 2011, Ivan Dryanovski, William Morris
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the CCNY Robotics Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*  This package uses Canonical Scan Matcher [1], written by
 *  Andrea Censi
 *
 *  [1] A. Censi, "An ICP variant using a point-to-line metric"
 *  Proceedings of the IEEE International Conference
 *  on Robotics and Automation (ICRA), 2008
 */

#include <laser_scan_matcher/laser_scan_matcher.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/assign.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer_interface.h>
namespace scan_tools
{
// see e.g. https://stackoverflow.com/a/40691657
constexpr std::chrono::duration<int64_t> LaserScanMatcher::DIAGNOSTICS_PERIOD;

LaserScanMatcher::LaserScanMatcher()
: Node("LaserScanMatcher")
, initialized_(false)
, received_imu_(false)
, received_odom_(false)
, received_vel_(false)
, last_loop_update_(rclcpp::Clock().now())
{
  RCLCPP_INFO(this->get_logger(), "%s\n","Starting LaserScanMatcher");
  
  // **** init parameters

  initParams();

  tf_listener_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  //tf_listener_buffer_->setUsingDedicatedThread(true);
  tf_listener_ =  std::make_shared<tf2_ros::TransformListener>(*tf_listener_buffer_);
  if(publish_tf_){
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }
  // **** state variables

  f2b_.setIdentity();
  f2b_kf_.setIdentity();
  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;

  // Initialize output_ vectors as Null for error-checking
  output_.cov_x_m = 0;
  output_.dx_dy1_m = 0;
  output_.dx_dy2_m = 0;

  // **** publishers

  if (publish_pose_)
  {
    pose_publisher_  = create_publisher<geometry_msgs::msg::Pose2D>(
      "pose2D", rclcpp::QoS(rclcpp::KeepLast(5)));
  }

  if (publish_pose_stamped_)
  {
    pose_stamped_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "pose_stamped", rclcpp::QoS(rclcpp::KeepLast(5)));
  }

  if (publish_pose_with_covariance_)
  {
    pose_with_covariance_publisher_  = create_publisher<geometry_msgs::msg::PoseWithCovariance>(
      "pose_with_covariance", rclcpp::QoS(rclcpp::KeepLast(5)));
  }

  if (publish_pose_with_covariance_stamped_)
  {
    pose_with_covariance_stamped_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pose_with_covariance_stamped", rclcpp::QoS(rclcpp::KeepLast(5)));
  }

  // *** subscribers

  if (use_cloud_input_)
  {
    std::string cloud_topic = params_.cloud_topic_name;
    cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic
      , 1
      , std::bind(&LaserScanMatcher::cloudCallback, this, std::placeholders::_1)
    );
  }
  else
  {
    std::string scan_topic = params_.scan_topic_name;
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic
      , 1
      , std::bind(&LaserScanMatcher::scanCallback, this, std::placeholders::_1)
    );
  }

  if (use_imu_)
  {
    std::string imu_topic = params_.imu_topic_name;
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic
      , 1
      , std::bind(&LaserScanMatcher::imuCallback, this, std::placeholders::_1)
    );
  }
  if (use_odom_)
  {
    std::string odom_topic = params_.odom_topic_name;
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic
      , 1
      , std::bind(&LaserScanMatcher::odomCallback, this, std::placeholders::_1)
    );
  }
  if (use_vel_)
  {
    std::string vel_topic = params_.vel_topic_name;
    //if (stamped_vel_)
    //  vel_stamped_subscriber_  = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    //      vel_topic
    //    , 1
    //    , std::bind(&LaserScanMatcher::velStmpCallback, this, std::placeholders::_1)
    //  );
    //else
      vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
          vel_topic
        , 1
        , std::bind(&LaserScanMatcher::velCallback, this, std::placeholders::_1)
      );
  }

  /// Diagnostics:
  diagnostics_ = std::make_shared<diagnostics_type>(this);
  status_ = std::make_shared<status_type>();
  //status_->velocity_hs = velocity_hs_;
  //status_->lock_hs = lock_hs_;

  diagnostics_timer_ = this->create_wall_timer(
    DIAGNOSTICS_PERIOD, [this]() -> void {
      updateDiagnostics();
    });
}

LaserScanMatcher::~LaserScanMatcher()
{
  RCLCPP_INFO(this->get_logger(), "%s\n","Destroying LaserScanMatcher");
}

std::shared_ptr<rclcpp::Node> LaserScanMatcher::get_node() {
    
    return node_;
}
//std::shared_ptr<rclcpp::Node> LaserScanMatcher::get_node() const {
//  const std::shared_ptr<rclcpp::Node> s(dynamic_cast<rclcpp::Node*>(this));
//  return s;
//}

void LaserScanMatcher::updateDiagnostics()
{
  //status_->priority = getLockPriority();
  status_-> scan_valid = output_.valid;
  status_-> match_duration = last_duration_;
  status_-> reading_age = reading_age_;
  diagnostics_->updateStatus(status_);
}


void LaserScanMatcher::initParams()
{

  node_.reset(dynamic_cast<rclcpp::Node*>(this));
  try
  {
    param_listener_ = std::make_shared<laser_scan_matcher::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    throw std::runtime_error (e.what());
  }
 
  //if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ =  params_.base_frame;
  //if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = params_.fixed_frame;

  // **** input type - laser scan, or point clouds?
  // if false, will subscribe to LaserScan msgs on /scan.
  // if true, will subscribe to PointCloud2 msgs on /cloud

  //if (!nh_private_.getParam ("use_cloud_input", use_cloud_input_))
    use_cloud_input_= params_.use_cloud_input;

  if (use_cloud_input_)
  {
    //if (!nh_private_.getParam ("cloud_range_min", cloud_range_min_))
      cloud_range_min_ = params_.cloud_range_min;
    //if (!nh_private_.getParam ("cloud_range_max", cloud_range_max_))
      cloud_range_max_ = params_.cloud_range_max;
    //if (!nh_private_.getParam ("cloud_res", cloud_res_))
      cloud_res_ = params_.cloud_res;

    input_.min_reading = cloud_range_min_;
    input_.max_reading = cloud_range_max_;
  }

  // **** keyframe params: when to generate the keyframe scan
  // if either is set to 0, reduces to frame-to-frame matching

  //if (!nh_private_.getParam ("kf_dist_linear", kf_dist_linear_))
    kf_dist_linear_ = params_.kf_dist_linear;
  //if (!nh_private_.getParam ("kf_dist_angular", kf_dist_angular_))
  if (params_.kf_dist_angular == 0)
    kf_dist_angular_ = 10.0 * (M_PI / 180.0);
  else {
    kf_dist_angular_ = params_.kf_dist_angular;
  }

  kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;

  // **** What predictions are available to speed up the ICP?
  // 1) imu - [theta] from imu yaw angle - /imu topic
  // 2) odom - [x, y, theta] from wheel odometry - /odom topic
  // 3) vel - [x, y, theta] from velocity predictor - see alpha-beta predictors - /vel topic
  // If more than one is enabled, priority is imu > odom > vel

  //if (!nh_private_.getParam ("use_imu", use_imu_))
    use_imu_ = params_.use_imu;
  //if (!nh_private_.getParam ("use_odom", use_odom_))
    use_odom_ = params_.use_odom;
  //if (!nh_private_.getParam ("use_vel", use_vel_))
    use_vel_ = params_.use_vel;

  // **** Are velocity input messages stamped?
  // if false, will subscribe to Twist msgs on /vel
  // if true, will subscribe to TwistStamped msgs on /vel
  //if (!nh_private_.getParam ("stamped_vel", stamped_vel_))
    stamped_vel_ = params_.stamped_vel;

  // **** How to publish the output?
  // tf (fixed_frame->base_frame),
  // pose message (pose of base frame in the fixed frame)

  //if (!nh_private_.getParam ("publish_tf", publish_tf_))
    publish_tf_ = params_.publish_tf;
  //if (!nh_private_.getParam ("publish_pose", publish_pose_))
    publish_pose_ = params_.publish_pose;
  //if (!nh_private_.getParam ("publish_pose_stamped", publish_pose_stamped_))
    publish_pose_stamped_ = params_.publish_pose_stamped;
  //if (!nh_private_.getParam ("publish_pose_with_covariance", publish_pose_with_covariance_))
    publish_pose_with_covariance_ = params_.publish_pose_with_covariance;
  //if (!nh_private_.getParam ("publish_pose_with_covariance_stamped", publish_pose_with_covariance_stamped_))
    publish_pose_with_covariance_stamped_ = params_.publish_pose_with_covariance_stamped;

  //if (!nh_private_.getParam("position_covariance", position_covariance_))
  if(params_.position_covariance.size()==0)
  {
    position_covariance_.resize(3);
    std::fill(position_covariance_.begin(), position_covariance_.end(), 1e-9);
  } else {
    position_covariance_.insert(position_covariance_.begin(), params_.position_covariance.begin(), params_.position_covariance.end()); 
  }

  //if (!nh_private_.getParam("orientation_covariance", orientation_covariance_))
  if(params_.orientation_covariance.size()==0)
  {
    orientation_covariance_.resize(3);
    std::fill(orientation_covariance_.begin(), orientation_covariance_.end(), 1e-9);
  }  else {
    orientation_covariance_.insert(orientation_covariance_.begin(), params_.orientation_covariance.begin(), params_.orientation_covariance.end()); 
  }
  // **** CSM parameters - comments copied from algos.h (by Andrea Censi)

  // Maximum angular displacement between scans
  //if (!nh_private_.getParam ("max_angular_correction_deg", input_.max_angular_correction_deg))
    input_.max_angular_correction_deg = params_.max_angular_correction_deg;

  // Maximum translation between scans (m)
  //if (!nh_private_.getParam ("max_linear_correction", input_.max_linear_correction))
    input_.max_linear_correction = params_.max_linear_correction;

  // Maximum ICP cycle iterations
  //if (!nh_private_.getParam ("max_iterations", input_.max_iterations))
    input_.max_iterations = params_.max_iterations;

  // A threshold for stopping (m)
  //if (!nh_private_.getParam ("epsilon_xy", input_.epsilon_xy))
    input_.epsilon_xy = params_.epsilon_xy;

  // A threshold for stopping (rad)
  //if (!nh_private_.getParam ("epsilon_theta", input_.epsilon_theta))
    input_.epsilon_theta = params_.epsilon_theta;

  // Maximum distance for a correspondence to be valid
  //if (!nh_private_.getParam ("max_correspondence_dist", input_.max_correspondence_dist))
    input_.max_correspondence_dist = params_.max_correspondence_dist;

  // Noise in the scan (m)
  //if (!nh_private_.getParam ("sigma", input_.sigma))
    input_.sigma = params_.sigma;

  // Use smart tricks for finding correspondences.
  //if (!nh_private_.getParam ("use_corr_tricks", input_.use_corr_tricks))
    input_.use_corr_tricks = params_.use_corr_tricks;

  // Restart: Restart if error is over threshold
  //if (!nh_private_.getParam ("restart", input_.restart))
    input_.restart = params_.restart;

  // Restart: Threshold for restarting
  //if (!nh_private_.getParam ("restart_threshold_mean_error", input_.restart_threshold_mean_error))
    input_.restart_threshold_mean_error = params_.restart_threshold_mean_error;

  // Restart: displacement for restarting. (m)
  //if (!nh_private_.getParam ("restart_dt", input_.restart_dt))
    input_.restart_dt = params_.restart_dt;

  // Restart: displacement for restarting. (rad)
  //if (!nh_private_.getParam ("restart_dtheta", input_.restart_dtheta))
    input_.restart_dtheta = params_.restart_dtheta;

  // Max distance for staying in the same clustering
  //if (!nh_private_.getParam ("clustering_threshold", input_.clustering_threshold))
    input_.clustering_threshold = params_.clustering_threshold;

  // Number of neighbour rays used to estimate the orientation
  //if (!nh_private_.getParam ("orientation_neighbourhood", input_.orientation_neighbourhood))
    input_.orientation_neighbourhood = params_.orientation_neighbourhood;

  // If 0, it's vanilla ICP
  //if (!nh_private_.getParam ("use_point_to_line_distance", input_.use_point_to_line_distance))
    input_.use_point_to_line_distance = params_.use_point_to_line_distance;

  // Discard correspondences based on the angles
  //if (!nh_private_.getParam ("do_alpha_test", input_.do_alpha_test))
    input_.do_alpha_test = params_.do_alpha_test;

  // Discard correspondences based on the angles - threshold angle, in degrees
  //if (!nh_private_.getParam ("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
    input_.do_alpha_test_thresholdDeg = params_.do_alpha_test_thresholdDeg;

  // Percentage of correspondences to consider: if 0.9,
  // always discard the top 10% of correspondences with more error
  //if (!nh_private_.getParam ("outliers_maxPerc", input_.outliers_maxPerc))
    input_.outliers_maxPerc = params_.outliers_maxPerc;

  // Parameters describing a simple adaptive algorithm for discarding.
  //  1) Order the errors.
  //  2) Choose the percentile according to outliers_adaptive_order.
  //     (if it is 0.7, get the 70% percentile)
  //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
  //     with the value of the error at the chosen percentile.
  //  4) Discard correspondences over the threshold.
  //  This is useful to be conservative; yet remove the biggest errors.
  //if (!nh_private_.getParam ("outliers_adaptive_order", input_.outliers_adaptive_order))
    input_.outliers_adaptive_order = params_.outliers_adaptive_order;

  //if (!nh_private_.getParam ("outliers_adaptive_mult", input_.outliers_adaptive_mult))
    input_.outliers_adaptive_mult = params_.outliers_adaptive_mult;

  // If you already have a guess of the solution, you can compute the polar angle
  // of the points of one scan in the new position. If the polar angle is not a monotone
  // function of the readings index, it means that the surface is not visible in the
  // next position. If it is not visible, then we don't use it for matching.
  //if (!nh_private_.getParam ("do_visibility_test", input_.do_visibility_test))
    input_.do_visibility_test = params_.do_visibility_test;

  // no two points in laser_sens can have the same corr.
  //if (!nh_private_.getParam ("outliers_remove_doubles", input_.outliers_remove_doubles))
    input_.outliers_remove_doubles = params_.outliers_remove_doubles;

  // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  //if (!nh_private_.getParam ("do_compute_covariance", input_.do_compute_covariance))
    input_.do_compute_covariance = params_.do_compute_covariance;

  // Checks that find_correspondences_tricks gives the right answer
  //if (!nh_private_.getParam ("debug_verify_tricks", input_.debug_verify_tricks))
    input_.debug_verify_tricks = params_.debug_verify_tricks;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
  // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
  //if (!nh_private_.getParam ("use_ml_weights", input_.use_ml_weights))
    input_.use_ml_weights = params_.use_ml_weights;

  // If 1, the field 'readings_sigma' in the second scan is used to weight the
  // correspondence by 1/sigma^2
  //if (!nh_private_.getParam ("use_sigma_weights", input_.use_sigma_weights))
    input_.use_sigma_weights = params_.use_sigma_weights;
}

void LaserScanMatcher::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  std::scoped_lock lock(mutex_);
  latest_imu_msg_ = *imu_msg;
  if (!received_imu_)
  {
    last_used_imu_msg_ = *imu_msg;
    received_imu_ = true;
  }
}

void LaserScanMatcher::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  std::scoped_lock lock(mutex_);
  latest_odom_msg_ = *odom_msg;
  if (!received_odom_)
  {
    last_used_odom_msg_ = *odom_msg;
    received_odom_ = true;
  }
}

void LaserScanMatcher::velCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  std::scoped_lock lock(mutex_);
  latest_vel_msg_ = *twist_msg;

  received_vel_ = true;
}

void LaserScanMatcher::velStmpCallback(const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg)
{
  std::scoped_lock lock(mutex_);
  latest_vel_msg_ = twist_msg->twist;

  received_vel_ = true;
}

void LaserScanMatcher::cloudCallback (const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  // **** if first scan, cache the tf from base to the scanner


  // PCL still uses boost::shared_ptr internally
  std::shared_ptr<PointCloudT> pcl_cloud = std::make_shared<PointCloudT>();

  pcl::fromROSMsg(*cloud, *pcl_cloud);
  //std_msgs::msg::Header cloud_header = pcl_conversions::fromPCL(pcl_cloud->header);
  std_msgs::msg::Header cloud_header = cloud->header;

  if (!initialized_)
  {
    // cache the static tf from base to laser
    if (!getBaseToLaserTf(cloud_header.frame_id))
    {
      RCLCPP_WARN(this->get_logger(),"Skipping scan\n");
      return;
    }

    PointCloudToLDP(pcl_cloud, prev_ldp_scan_);
    last_icp_time_ = cloud_header.stamp;
    initialized_ = true;
  }

  LDP curr_ldp_scan;
  PointCloudToLDP(pcl_cloud, curr_ldp_scan);
  processScan(curr_ldp_scan, cloud_header.stamp);
}

void LaserScanMatcher::scanCallback (const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  // **** if first scan, cache the tf from base to the scanner

  if (!initialized_)
  {
    createCache(scan_msg);    // caches the sin and cos of all angles

    // cache the static tf from base to laser
    if (!getBaseToLaserTf(scan_msg->header.frame_id))
    {
      RCLCPP_WARN(this->get_logger(),"Skipping scan");
      return;
    }
    laserScanToLDP(scan_msg, prev_ldp_scan_);
    last_icp_time_ = scan_msg->header.stamp;
    initialized_ = true;
  }

  LDP curr_ldp_scan;
  laserScanToLDP(scan_msg, curr_ldp_scan);
  processScan(curr_ldp_scan, scan_msg->header.stamp);
}

void LaserScanMatcher::processScan(LDP& curr_ldp_scan, const rclcpp::Time& time)
{
  const auto start = std::chrono::system_clock::now();
  reading_age_ = (rclcpp::Clock().now() -last_loop_update_).seconds();
  last_loop_update_ = rclcpp::Clock().now();
  // CSM is used in the following way:
  // The scans are always in the laser frame
  // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
  // The new scan (currLDPScan) has a pose equal to the movement
  // of the laser in the laser frame since the last scan
  // The computed correction is then propagated using the tf machinery

  prev_ldp_scan_->odometry[0] = 0.0;
  prev_ldp_scan_->odometry[1] = 0.0;
  prev_ldp_scan_->odometry[2] = 0.0;

  prev_ldp_scan_->estimate[0] = 0.0;
  prev_ldp_scan_->estimate[1] = 0.0;
  prev_ldp_scan_->estimate[2] = 0.0;

  prev_ldp_scan_->true_pose[0] = 0.0;
  prev_ldp_scan_->true_pose[1] = 0.0;
  prev_ldp_scan_->true_pose[2] = 0.0;

  input_.laser_ref  = prev_ldp_scan_;
  input_.laser_sens = curr_ldp_scan;

  // **** estimated change since last scan

  double dt = (time - last_icp_time_).seconds();
  double pr_ch_x, pr_ch_y, pr_ch_a;
  getPrediction(pr_ch_x, pr_ch_y, pr_ch_a, dt);

  // the predicted change of the laser's position, in the fixed frame

  tf2::Transform pr_ch;
  createTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, pr_ch);

  // account for the change since the last kf, in the fixed frame

  pr_ch = pr_ch * (f2b_ * f2b_kf_.inverse());

  // the predicted change of the laser's position, in the laser frame

  tf2::Transform pr_ch_l;
  pr_ch_l = laser_to_base_ * f2b_.inverse() * pr_ch * f2b_ * base_to_laser_ ;

  input_.first_guess[0] = pr_ch_l.getOrigin().getX();
  input_.first_guess[1] = pr_ch_l.getOrigin().getY();
  input_.first_guess[2] = tf2::getYaw(pr_ch_l.getRotation());
  // If they are non-Null, free covariance gsl matrices to avoid leaking memory
  if (output_.cov_x_m)
  {
    gsl_matrix_free(output_.cov_x_m);
    output_.cov_x_m = 0;
  }
  if (output_.dx_dy1_m)
  {
    gsl_matrix_free(output_.dx_dy1_m);
    output_.dx_dy1_m = 0;
  }
  if (output_.dx_dy2_m)
  {
    gsl_matrix_free(output_.dx_dy2_m);
    output_.dx_dy2_m = 0;
  }

  // *** scan match - using point to line icp from CSM

  sm_icp(&input_, &output_);
  tf2::Transform corr_ch;

  if (output_.valid)
  {

    // the correction of the laser's position, in the laser frame
    tf2::Transform corr_ch_l;
    createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);

    // the correction of the base's position, in the base frame
    corr_ch = base_to_laser_ * corr_ch_l * laser_to_base_;

    // update the pose in the world frame
    f2b_ = f2b_kf_ * corr_ch;

    // **** publish

    if (publish_pose_)
    {
      // unstamped Pose2D message
      geometry_msgs::msg::Pose2D::SharedPtr pose_msg;
      pose_msg = std::make_shared<geometry_msgs::msg::Pose2D>();
      pose_msg->x = f2b_.getOrigin().getX();
      pose_msg->y = f2b_.getOrigin().getY();
      pose_msg->theta = tf2::getYaw(f2b_.getRotation());
      pose_publisher_->publish(*pose_msg);
    }
    if (publish_pose_stamped_)
    {
      // stamped Pose message
      geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped_msg;
      pose_stamped_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();

      pose_stamped_msg->header.stamp    = time;
      pose_stamped_msg->header.frame_id = fixed_frame_;
      tf2::toMsg(f2b_, pose_stamped_msg->pose);
      //tf::poseTFToMsg(f2b_, pose_stamped_msg->pose);

      pose_stamped_publisher_->publish(*pose_stamped_msg);
    }
    if (publish_pose_with_covariance_)
    {
      // unstamped PoseWithCovariance message
      geometry_msgs::msg::PoseWithCovariance::SharedPtr pose_with_covariance_msg;
      pose_with_covariance_msg = std::make_shared<geometry_msgs::msg::PoseWithCovariance>();
      tf2::toMsg(f2b_, pose_with_covariance_msg->pose);
      //tf::poseTFToMsg(f2b_, pose_with_covariance_msg->pose);

      if (input_.do_compute_covariance)
      {
        pose_with_covariance_msg->covariance = boost::assign::list_of
          (gsl_matrix_get(output_.cov_x_m, 0, 0)) (0)  (0)  (0)  (0)  (0)
          (0)  (gsl_matrix_get(output_.cov_x_m, 0, 1)) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (gsl_matrix_get(output_.cov_x_m, 0, 2));
      }
      else
      {
        pose_with_covariance_msg->covariance = boost::assign::list_of
          (static_cast<double>(position_covariance_[0])) (0)  (0)  (0)  (0)  (0)
          (0)  (static_cast<double>(position_covariance_[1])) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[2]));
      }

      pose_with_covariance_publisher_->publish(*pose_with_covariance_msg);
    }
    if (publish_pose_with_covariance_stamped_)
    {
      // stamped Pose message
      geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_with_covariance_stamped_msg;
      pose_with_covariance_stamped_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

      pose_with_covariance_stamped_msg->header.stamp    = time;
      pose_with_covariance_stamped_msg->header.frame_id = fixed_frame_;

      tf2::toMsg(f2b_, pose_with_covariance_stamped_msg->pose.pose);
      //tf2::poseTFToMsg(f2b_, pose_with_covariance_stamped_msg->pose.pose);

      if (input_.do_compute_covariance)
      {
        pose_with_covariance_stamped_msg->pose.covariance = boost::assign::list_of
          (gsl_matrix_get(output_.cov_x_m, 0, 0)) (0)  (0)  (0)  (0)  (0)
          (0)  (gsl_matrix_get(output_.cov_x_m, 0, 1)) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (gsl_matrix_get(output_.cov_x_m, 0, 2));
      }
      else
      {
        pose_with_covariance_stamped_msg->pose.covariance = boost::assign::list_of
          (static_cast<double>(position_covariance_[0])) (0)  (0)  (0)  (0)  (0)
          (0)  (static_cast<double>(position_covariance_[1])) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[2]));
      }

      pose_with_covariance_stamped_publisher_->publish(*pose_with_covariance_stamped_msg);
    }
    if (publish_tf_)
    {
      tf2::TimePoint ptime = tf2_ros::fromMsg(time);
      tf2::Stamped<tf2::Transform> transform_msg (f2b_, ptime, base_frame_);
      geometry_msgs::msg::TransformStamped transf = tf2::toMsg(transform_msg);
      transf.child_frame_id = fixed_frame_;
      tf_broadcaster_->sendTransform (transf);
    }
  }
  else
  {
    corr_ch.setIdentity();
    RCLCPP_WARN(this->get_logger(), "Error in scan matching\n");
  }

  // **** swap old and new
  if (newKeyframeNeeded(corr_ch))
  {
    // generate a keyframe
    ld_free(prev_ldp_scan_);
    prev_ldp_scan_ = curr_ldp_scan;
    f2b_kf_ = f2b_;
  }
  else
  {
    ld_free(curr_ldp_scan);
  }

  last_icp_time_ = time;

  // **** statistics
  
  last_duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count();
  RCLCPP_DEBUG(this->get_logger(), "Scan matcher total duration: %.1f ms", last_duration_);
}

bool LaserScanMatcher::newKeyframeNeeded(const tf2::Transform& d)
{
  if (fabs(tf2::getYaw(d.getRotation())) > kf_dist_angular_) return true;

  double x = d.getOrigin().getX();
  double y = d.getOrigin().getY();
  if (x*x + y*y > kf_dist_linear_sq_) return true;

  return false;
}

void LaserScanMatcher::PointCloudToLDP(const PointCloudT::ConstPtr& cloud,
                                             LDP& ldp)
{
  double max_d2 = cloud_res_ * cloud_res_;

  PointCloudT cloud_f;

  cloud_f.points.push_back(cloud->points[0]);

  for (unsigned int i = 1; i < cloud->points.size(); ++i)
  {
    const PointT& pa = cloud_f.points[cloud_f.points.size() - 1];
    const PointT& pb = cloud->points[i];

    double dx = pa.x - pb.x;
    double dy = pa.y - pb.y;
    double d2 = dx*dx + dy*dy;

    if (d2 > max_d2)
    {
      cloud_f.points.push_back(pb);
    }
  }

  unsigned int n = cloud_f.points.size();

  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame
    if (is_nan(cloud_f.points[i].x) || is_nan(cloud_f.points[i].y))
    {
      RCLCPP_WARN(this->get_logger(),"Laser Scan Matcher: Cloud input contains NaN values. \
                Please use a filtered cloud input.\n");
    }
    else
    {
      double r = sqrt(cloud_f.points[i].x * cloud_f.points[i].x +
                      cloud_f.points[i].y * cloud_f.points[i].y);

      if (r > cloud_range_min_ && r < cloud_range_max_)
      {
        ldp->valid[i] = 1;
        ldp->readings[i] = r;
      }
      else
      {
        ldp->valid[i] = 0;
        ldp->readings[i] = -1;  // for invalid range
      }
    }

    ldp->theta[i] = atan2(cloud_f.points[i].y, cloud_f.points[i].x);
    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void LaserScanMatcher::laserScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg,
                                            LDP& ldp)
{
  unsigned int n = scan_msg->ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame

    double r = scan_msg->ranges[i];

    if (r > scan_msg->range_min && r < scan_msg->range_max)
    {
      // fill in laser scan data

      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }

    ldp->theta[i]    = scan_msg->angle_min + i * scan_msg->angle_increment;

    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void LaserScanMatcher::createCache (const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  a_cos_.clear();
  a_sin_.clear();

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
    a_cos_.push_back(cos(angle));
    a_sin_.push_back(sin(angle));
  }

  input_.min_reading = scan_msg->range_min;
  input_.max_reading = scan_msg->range_max;
}

bool LaserScanMatcher::getBaseToLaserTf (const std::string& frame_id)
{
  rclcpp::Time t = now();
  tf2::Stamped<tf2::Transform> base_to_laser_tf; //tf2::StampedTransform
  geometry_msgs::msg::TransformStamped laser_pose_msg;
  try
  {
//    tf_listener_buffer_->waitForTransform(
//      base_frame_, frame_id, t, rclcpp::Duration::from_seconds(1.0));
    laser_pose_msg = tf_listener_buffer_->lookupTransform (
      base_frame_, frame_id, t, rclcpp::Duration::from_seconds(1.0));
    
    base_to_laser_tf.setOrigin(tf2::Vector3(laser_pose_msg.transform.translation.x,
                                              laser_pose_msg.transform.translation.y,
                                              laser_pose_msg.transform.translation.z)
                              );
    tf2::Quaternion q(laser_pose_msg.transform.rotation.x,
                        laser_pose_msg.transform.rotation.y,
                        laser_pose_msg.transform.rotation.z,
                        laser_pose_msg.transform.rotation.w
                      );
    base_to_laser_tf.setRotation(q);
  }
  catch (tf2::TransformException & ex)
  {
    RCLCPP_WARN(this->get_logger(), "Could not get initial transform from base %s to laser frame %s , %s \n",base_frame_.c_str() , frame_id.c_str(), ex.what());
    return false;
  }
  base_to_laser_ = base_to_laser_tf;
  laser_to_base_ = base_to_laser_.inverse();

  return true;
}

// returns the predicted change in pose (in fixed frame)
// since the last time we did icp
void LaserScanMatcher::getPrediction(double& pr_ch_x, double& pr_ch_y,
                                     double& pr_ch_a, double dt)
{
  std::scoped_lock lock(mutex_);

  // **** base case - no input available, use zero-motion model
  pr_ch_x = 0.0;
  pr_ch_y = 0.0;
  pr_ch_a = 0.0;

  // **** use velocity (for example from ab-filter)
  if (use_vel_)
  {
    pr_ch_x = dt * latest_vel_msg_.linear.x;
    pr_ch_y = dt * latest_vel_msg_.linear.y;
    pr_ch_a = dt * latest_vel_msg_.angular.z;

    if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
    else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;
  }

  // **** use wheel odometry
  if (use_odom_ && received_odom_)
  {
    pr_ch_x = latest_odom_msg_.pose.pose.position.x -
              last_used_odom_msg_.pose.pose.position.x;

    pr_ch_y = latest_odom_msg_.pose.pose.position.y -
              last_used_odom_msg_.pose.pose.position.y;

    pr_ch_a = tf2::getYaw(latest_odom_msg_.pose.pose.orientation) -
              tf2::getYaw(last_used_odom_msg_.pose.pose.orientation);

    if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
    else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;

    last_used_odom_msg_ = latest_odom_msg_;
  }

  // **** use imu
  if (use_imu_ && received_imu_)
  {
    pr_ch_a = tf2::getYaw(latest_imu_msg_.orientation) -
              tf2::getYaw(last_used_imu_msg_.orientation);

    if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
    else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;

    last_used_imu_msg_ = latest_imu_msg_;
  }
}

void LaserScanMatcher::createTfFromXYTheta(
  double x, double y, double theta, tf2::Transform& t)
{
  t.setOrigin(tf2::Vector3(x, y, 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}

} // namespace scan_tools
