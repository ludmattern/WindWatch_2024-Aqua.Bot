#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class AquabotNode : public rclcpp::Node
{
public:
  AquabotNode() : Node("aquabot_node_cpp"), m_target_pos(1.0)
  {
    RCLCPP_INFO(this->get_logger(), "AquabotNode started!");

    // Créer des publishers pour les positions des thrusters
    m_thrustersLeftPos_pub = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/pos", 10);
    m_thrustersRightPos_pub = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/pos", 10);

    // Créer un timer pour publier périodiquement la position des moteurs
    using namespace std::chrono_literals;
    m_timer = this->create_wall_timer(2000ms, [this]() { timer_callback(); });
  }

private:
  // Callback appelé par le timer toutes les 2 secondes
  void timer_callback()
  {
    m_target_pos = -m_target_pos; // Inverse la position cible à chaque appel

    // Créer un message de type Float64 et le publier sur les deux moteurs
    auto pos_msg = std_msgs::msg::Float64();
    pos_msg.data = m_target_pos;

    m_thrustersLeftPos_pub->publish(pos_msg);
    m_thrustersRightPos_pub->publish(pos_msg);

    RCLCPP_INFO(this->get_logger(), "Published position: %f", m_target_pos);
  }

  // Publishers pour la position des moteurs gauche et droit
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_thrustersLeftPos_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_thrustersRightPos_pub;

  // Timer qui appelle périodiquement la fonction de callback
  rclcpp::TimerBase::SharedPtr m_timer;

  // Variable pour la position cible (inversée à chaque cycle)
  double m_target_pos;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AquabotNode>());
  rclcpp::shutdown();
  return 0;
}
