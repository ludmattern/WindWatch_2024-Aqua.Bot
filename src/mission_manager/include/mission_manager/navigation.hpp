// action/navigation.hpp

#ifndef MISSION_MANAGER__ACTION__NAVIGATION_HPP_
#define MISSION_MANAGER__ACTION__NAVIGATION_HPP_

#include <rosidl_generator_cpp/action/goal.hpp>
#include <rosidl_generator_cpp/action/result.hpp>
#include <rosidl_generator_cpp/action/feedback.hpp>
#include <nav_msgs/msg/path.hpp>

namespace mission_manager
{
namespace action
{

struct Navigation_Goal
{
    nav_msgs::msg::Path path;
};

struct Navigation_Result
{
    bool success;
};

struct Navigation_Feedback
{
    float progress; // Progression en pourcentage
};

struct Navigation
{
    using Goal = Navigation_Goal;
    using Result = Navigation_Result;
    using Feedback = Navigation_Feedback;
};

} // namespace action
} // namespace mission_manager

#endif  // MISSION_MANAGER__ACTION__NAVIGATION_HPP_
