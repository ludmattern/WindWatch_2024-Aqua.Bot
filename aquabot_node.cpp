#include "AquabotNode.hpp"

AquabotNode::AquabotNode() : Node("aquabot_node_cpp"), m_target_pos(1.0), m_current_pos(0.0), m_target_thrust(1000.0)
{
RCLCPP_INFO(this->get_logger(), "Hello world from aquabot_node_cpp!");

    // Create publishers for thrusters
    m_thrustersLeftPos_pub = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/pos", 100);
    m_thrustersRightPos_pub = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/pos", 10);
    m_thrustersLeftThrust_pub = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/thrust", 10);
    m_thrustersRightThrust_pub = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/thrust", 10);

    // Create timers for engine update and position update using lambdas
    m_tf2_timer = this->create_wall_timer(100ms, [this]() { update_engine_position(); });
    m_timer = this->create_wall_timer(10s, [this]() { timer_callback(); });

    // Create tf2 buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void AquabotNode::control_motors_powers(double thrust, double position)
{
    // Create and set messages for thrust and position
    auto thrust_msg = std_msgs::msg::Float64();
    thrust_msg.data = thrust;



    // Publish thrust to both thrusters
    m_thrustersLeftThrust_pub->publish(thrust_msg);
    m_thrustersRightThrust_pub->publish(thrust_msg);



    // Log the control information
    RCLCPP_INFO(this->get_logger(), "Control motors: thrust = '%f', position = '%f'", thrust, position);
}

void AquabotNode::control_motors_pos(double position)
{
    auto pos_msg = std_msgs::msg::Float64();
    pos_msg.data = position;
    // Publish position to both thrusters
    m_thrustersLeftPos_pub->publish(pos_msg);
    m_thrustersRightPos_pub->publish(pos_msg);
}

void AquabotNode::timer_callback()
{
    m_target_pos = -m_target_pos; // Toggle target position
    double thrust = m_target_thrust; // Set desired thrust
    double position = m_target_pos; // Set desired position

    control_motors_pos(position);
    // Use the new control function
    control_motors_powers(thrust, position);

    RCLCPP_INFO(this->get_logger(), "New target pos: '%f', thrust: '%f'", position, thrust);
}

void AquabotNode::update_engine_position()
{
    // Get the transform (relative engine position from aquabot)
    geometry_msgs::msg::TransformStamped leftEngineTf;
    try
    {
        leftEngineTf = tf_buffer_->lookupTransform("aquabot", "aquabot/aquabot/left_engine_link", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return;
    }

    // Convert to RPY
    tf2::Quaternion q(
        leftEngineTf.transform.rotation.x,
        leftEngineTf.transform.rotation.y,
        leftEngineTf.transform.rotation.z,
        leftEngineTf.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Update current position
    m_current_pos = yaw;
    RCLCPP_INFO(this->get_logger(), "Left engine current pos: '%f'", m_current_pos);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AquabotNode>());
    rclcpp::shutdown();
    return 0;
}
