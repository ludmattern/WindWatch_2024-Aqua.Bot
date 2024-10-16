#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

#define MAX_THRUSTER_POS M_PI/2

class AquabotNode : public rclcpp::Node
{
  public:
	AquabotNode();
    //void setPower(double thrust, double position);
    
private:
	void timer_callback();
	void update_engine_position();
    void control_motors_powers(double thrust, double position);
    void control_motors_pos(double position);
    void timer_callback(double thrust, double position);

    rclcpp::TimerBase::SharedPtr m_tf2_timer;
    rclcpp::TimerBase::SharedPtr m_timer;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_thrustersLeftPos_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_thrustersRightPos_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_thrustersLeftThrust_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_thrustersRightThrust_pub;

	float m_target_pos;
    float m_current_pos;
    float m_target_thrust;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};
