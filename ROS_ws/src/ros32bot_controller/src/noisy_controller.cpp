#include "ros32bot_controller/noisy_controller.hpp"
#include "Eigen/Geometry"
#include <tf2/LinearMath/Quaternion.hpp>

#include <random>

using std::placeholders::_1;

NoisyController::NoisyController(const std::string &name)
    : Node(name),
    left_wheel_prev_pos_(0),
    right_wheel_prev_pos_(0),
    x_(0), y_(0), theta_(0),
    noise_generator_(std::random_device{}()),
    left_encoder_noise_(0.0, 0.05),
    right_encoder_noise_(0.0, 0.05)
{
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation",0.17);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius: " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel separation: " << wheel_separation_);

    prev_time_ = get_clock()->now();

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&NoisyController::jointCallback, this, _1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("ros32bot_controller/odom_noisy",10);

    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint_ekf";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 0.0;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this); //pass in *this cuz the broadcaster
                                                                                    //uses this node's structure to set up its broadcasting structure
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint_noisy";
    

}


void NoisyController::jointCallback(const sensor_msgs::msg::JointState & msg)
{
    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;
    double dt_sec = dt.seconds();

    // Noise is scaled by sqrt(dt) (proper discretization of a continuous-time noise
    // process) so the accumulated heading/position error depends on elapsed time, not
    // on how fast /joint_states happens to be published. Injecting a fixed-magnitude
    // draw every callback regardless of dt made the odometry noise -- and its effect on
    // theta_, which is a small difference of two such noisy terms -- scale with the
    // control loop rate instead of with real elapsed time.
    // Clamp: a non-positive dt (first callback, or a clock jump) must not inject sqrt()
    // of a negative number and poison theta_/x_/y_ with NaN for the rest of the run.
    double noise_dt = dt_sec > 0.0 ? dt_sec : 0.0;
    double left_noise = left_encoder_noise_(noise_generator_) * sqrt(noise_dt);
    double right_noise = right_encoder_noise_(noise_generator_) * sqrt(noise_dt);

    double wheel_encoder_left = msg.position.at(1) + left_noise;
    double wheel_encoder_right = msg.position.at(0) + right_noise;

    double dp_left = wheel_encoder_left - left_wheel_prev_pos_;
    double dp_right = wheel_encoder_right - right_wheel_prev_pos_;

    left_wheel_prev_pos_ = msg.position.at(1);
    right_wheel_prev_pos_ = msg.position.at(0);
    prev_time_ = msg_time;

    double fi_left = dp_left / dt_sec; //rot vel
    double fi_right = dp_right / dt_sec;

    double linear_vel =  ( wheel_radius_* fi_right + wheel_radius_ * fi_left) / 2;
    double angular_vel = (wheel_radius_ * fi_right - wheel_radius_ * fi_left) / wheel_separation_;

    double d_s = (wheel_radius_ * dp_right + wheel_radius_ * dp_left)/2;
    double d_theta = (wheel_radius_ * dp_right - wheel_radius_ * dp_left)/wheel_separation_;

    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    tf2::Quaternion q;
    q.setRPY(0,0,theta_);
    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();
    odom_msg_.header.stamp = get_clock()->now();
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.twist.twist.linear.x = linear_vel;
    odom_msg_.twist.twist.angular.z = angular_vel;

    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.x();
    transform_stamped_.transform.rotation.y = q.y();
    transform_stamped_.transform.rotation.z = q.z();
    transform_stamped_.transform.rotation.w = q.w();
    transform_stamped_.header.stamp  = get_clock()->now();

    RCLCPP_INFO_STREAM(get_logger(), "linear  vel: " << linear_vel << "  , angular vel: " << angular_vel);
    RCLCPP_INFO_STREAM(get_logger(),  "theta: " << theta_ << " x: " << x_ << ", y: " << y_);
    
    odom_pub_->publish(odom_msg_);
    transform_broadcaster_->sendTransform(transform_stamped_);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<NoisyController>("noisy_controller");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}