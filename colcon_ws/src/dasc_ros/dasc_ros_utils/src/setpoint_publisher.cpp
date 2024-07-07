
#include "dasc_ros_utils/setpoint_publisher.hpp"

SetpointPublisher::SetpointPublisher() : Node("setpoint_publisher") {
  // declare parameters

  using std::placeholders::_1;
  using namespace std::chrono_literals;

  px4_robot_name_ =
      this->declare_parameter<std::string>("px4_robot_name", "px4_1");

  px4_world_frame_ = this->declare_parameter<std::string>("px4_local_frame",
                                                          "vicon/world/NED");

  publish_rate_ = this->declare_parameter<double>("publish_rate_hz", publish_rate_);
  skip_dt_ms_ = this->declare_parameter<double>("skip_dt_ms", skip_dt_ms_);
  terminal_yaw_freq_ = this-> declare_parameter<double>("terminal_yaw_freq_hz", terminal_yaw_freq_);
  terminal_yaw_amplitude_ = this->declare_parameter<double>("terminal_yaw_amplitude_rad", terminal_yaw_amplitude_);
  terminal_z_freq_ = this-> declare_parameter<double>("terminal_z_freq_hz", terminal_z_freq_);
  terminal_z_amplitude_ = this->declare_parameter<double>("terminal_z_amplitude_m", terminal_z_amplitude_);

  trajectory_topic_ =
      this->declare_parameter<std::string>("trajectory_topic", "trajectory");

  // construct the tf buffer
  std::chrono::duration<int> buffer_timeout(1);
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // create a publisher of the px4 setpoint
  pub_setpoint_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      px4_robot_name_ + "/fmu/in/trajectory_setpoint", 10);

  pub_traj_msg_viz_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
		  px4_robot_name_ + "/viz/trajectory", 10);

  // subscribe to dasc_msgs/DITrajectory
  traj_sub_.subscribe(this, trajectory_topic_);
  tf2_filter_ =
      std::make_shared<tf2_ros::MessageFilter<dasc_msgs::msg::DITrajectory>>(
          traj_sub_, *tf2_buffer_, px4_world_frame_, 100,
          this->get_node_logging_interface(), this->get_node_clock_interface(),
          buffer_timeout);
  tf2_filter_->registerCallback(&SetpointPublisher::trajectory_callback, this);

  // initialize the timer
  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_), 
      std::bind(&SetpointPublisher::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "starting setpoint publisher node");
}

px4_msgs::msg::TrajectorySetpoint
SetpointPublisher::interpolate_trajectory(int index, double delta,
                                          bool is_last) {
  // returns the trajectory setpoint at index + a small amount of time delta
  // assumes that all the relevant indices exist

  px4_msgs::msg::TrajectorySetpoint msg;
  msg.raw_mode = false;

  auto target_pose = traj_.poses[index];
  auto target_vel = traj_.twists[index];
  auto target_acc = traj_.accelerations[index];

  {
    double p = target_pose.position.x;
    double v = is_last ? 0.0 : target_vel.linear.x;
    double a = is_last ? 0.0 : target_acc.linear.x;

    msg.position[0] = p + v * delta + 0.5 * a * delta * delta;
    msg.velocity[0] = v + a * delta;
    msg.acceleration[0] = a;
    msg.jerk[0] = 0.0;
  }

  {
    double p = target_pose.position.y;
    double v = is_last ? 0.0 : target_vel.linear.y;
    double a = is_last ? 0.0 : target_acc.linear.y;

    msg.position[1] = p + v * delta + 0.5 * a * delta * delta;
    msg.velocity[1] = v + a * delta;
    msg.acceleration[1] = a;
    msg.jerk[1] = 0.0;
  }

  {
    double p = target_pose.position.z;
    double v = is_last ? 0.0 : target_vel.linear.z;
    double a = is_last ? 0.0 : target_acc.linear.z;


    msg.position[2] = p + v * delta + 0.5 * a * delta * delta;
    msg.velocity[2] = v + a * delta;
    msg.acceleration[2] = a;
    msg.jerk[2] = 0.0;
    
    if (is_last) 
    {
	    // add some up-down motion
	    double omega = 2* M_PI * terminal_z_freq_;
	    double add_p = terminal_z_amplitude_ * std::sin(omega * delta);
	    double add_v = omega * terminal_z_amplitude_ * std::cos(omega * delta);

	    msg.position[2] += add_p;
	    msg.velocity[2] += add_v;

    }
  }

  // angular
  { 

    msg.yawspeed = is_last ? 0.0 : target_vel.angular.z; 
    msg.yaw = tf2::getYaw(target_pose.orientation) + msg.yawspeed * delta;
    if (is_last) {
	    double terminal_omega = 2.0 * M_PI * terminal_yaw_freq_;

	    double terminal_yaw = terminal_yaw_amplitude_ * std::sin(terminal_omega * delta);
	    double terminal_yawspeed = terminal_omega * terminal_yaw_amplitude_ * std::cos(terminal_omega * delta);

	    msg.yaw += terminal_yaw;
	    msg.yawspeed += terminal_yawspeed;
    }
	    
  }

  return msg;
}

void SetpointPublisher::timer_callback() {
  
  //debug_timer_callback();
  //
  if (!traj_is_initialized) {
    RCLCPP_WARN_THROTTLE(get_logger(), *(get_clock()), 1000.0, "waiting for traj msg on topic %s...", traj_sub_.getTopic().c_str());
    return;
  }

  timer::Timer callbackTimer("setpoint publisher", false);

  size_t N = traj_.poses.size(); 
  bool is_last = false;

  // if we got here, it means that there is a message for us to parse
  auto now = this->get_clock()->now();
  double t = (now - traj_.header.stamp).seconds();

  // query at t + dt into the future
  t += 0.001 * skip_dt_ms_; 

  int index = t / traj_.dt; // integer division rounds down

  // RCLCPP_INFO(get_logger(), "index: %d, tau: %f", index, tau);

  if (index < 0) {
    RCLCPP_WARN(this->get_logger(), "woops, message is for the future");
    return; // not yet time to process this message
  }
  if (index >= int(N)) {
    index = N - 1;
    is_last = true;
  }

  // determine how far between two messages it is
  double tau = t - index * traj_.dt;

  auto msg = interpolate_trajectory(index, tau, is_last);

  pub_setpoint_->publish(msg);

  constexpr float kTimeBetweenDebugMessages = 1000.0; // ms
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kTimeBetweenDebugMessages, callbackTimer.log().c_str());

}

void SetpointPublisher::debug_timer_callback() {

  // get the latest quad state
  auto now = get_clock() -> now();

  // get the latest quad state
  geometry_msgs::msg::TransformStamped trans;
  try {
    trans = tf2_buffer_ -> lookupTransform("vicon/world", "vicon/px4_1/px4_1", tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(get_logger(), "could not tf: %s", ex.what());
    return;
  }

  // see time time delta
  RCLCPP_INFO(get_logger(), "last state was %f seconds ago", (now - trans.header.stamp).seconds());

  // get the relative time of where the quad should be
  double t = (rclcpp::Time(trans.header.stamp) - rclcpp::Time(traj_.header.stamp)).seconds();
  
  RCLCPP_INFO(get_logger(), "rel time:  %f seconds", t); 
  
  size_t N = traj_.poses.size(); 
  bool is_last = false;

  int index = t / traj_.dt; // integer division rounds down
  double tau = t - index * traj_.dt;

  if (index < 0) {
    RCLCPP_WARN(this->get_logger(), "woops, tried to query quad state before traj msg");
    return; 
  }
  if (index >= int(N)) {
    index = N - 1;
    is_last = true;
  }

  auto msg = interpolate_trajectory(index, tau, is_last);  // this is the expected state at this time

  // print the delta with the true state:
  RCLCPP_INFO(get_logger(), 
      "Expected: (%f, %f, %f) :: True: (%f, %f, %f)",
      msg.position[1], msg.position[0], -msg.position[2],
      trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z);

}

void SetpointPublisher::trajectory_callback(
    const dasc_msgs::msg::DITrajectory::SharedPtr msg) {

  // check the message 

  // get the total length of the messages
  size_t N_poses = msg->poses.size();
  size_t N_twists = msg->twists.size();
  size_t N_accels = msg->accelerations.size();
  size_t N = N_poses;

  if (N_poses <= 1) {
    RCLCPP_DEBUG(get_logger(), "trajectory msg has no poses.");
    return;
  }

  if ( (N_poses != N) || (N_twists != N) || (N_accels != N) ) {
    RCLCPP_DEBUG(get_logger(), "trajectory msg has incorrect number of poses");
    return;
  }


  // passed all checks

  geometry_msgs::msg::TransformStamped trans;

  try {
    trans = tf2_buffer_->lookupTransform(px4_world_frame_, msg->header.frame_id,
                                         msg->header.stamp);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Failure %s\n", ex.what());
    return;
  }

  // since the transform was successful we can save the trajectory callback
  tf2::doTransform(*msg, traj_, trans);

  traj_is_initialized = true;

  // // // now publish the pose array for visualizations
  // geometry_msgs::msg::PoseArray viz_msg;
  // viz_msg.header = traj_.header;
  // viz_msg.header.frame_id = px4_world_frame_;
  // viz_msg.poses = traj_.poses;
  // pub_traj_msg_viz_ -> publish(viz_msg);
  //
  //

}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetpointPublisher>());
  rclcpp::shutdown();
  return 0;
}
