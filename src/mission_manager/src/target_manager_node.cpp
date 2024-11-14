#include "mission_manager/target_manager_node.hpp"

TargetManagerNode::TargetManagerNode() : Node("target_manager_node")
{
    RCLCPP_INFO(this->get_logger(), "Target Manager Node has started");
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    
    
    TargetPath_Client_ = this->create_client<navigation::srv::Path>("/navigation/path");
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TargetManagerNode::launch, this));
}

void TargetManagerNode::launch()
{
    if (!this->TargetPath_Client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(this->get_logger(), "Service '/navigation/path' not available");
        return;
    }
    timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Service is available. Sending request...");

    auto request = std::make_shared<navigation::srv::Path::Request>();
    auto future = this->TargetPath_Client_->async_send_request(
        request,
        std::bind(&TargetManagerNode::service_response_callback, this, std::placeholders::_1)
    );
}

void TargetManagerNode::service_response_callback(
    rclcpp::Client<navigation::srv::Path>::SharedFuture future)
{
    RCLCPP_INFO(this->get_logger(), "Received response from service");

    auto response = future.get();

    if (response->path.poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Received empty poses from service. Retrying...");
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TargetManagerNode::launch, this));
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Processing received path");
        this->PathPlan(response->path);
        if (wind_data_.nb_wind == 0)
            this->TmaPosRegister(response->pose_array);
    }
}

void TargetManagerNode::TmaPosRegister(geometry_msgs::msg::PoseArray msg)
{
    if (msg.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received message without poses.");
        return;
    }
    wind_data_.wind.poses.clear();
    wind_data_.wind.poses.reserve(msg.poses.size());
    wind_data_.status.clear();
    wind_data_.status.resize(msg.poses.size(), false);
    wind_data_.qr.clear();
    wind_data_.qr.resize(msg.poses.size(), std_msgs::msg::String());

    for (size_t i = 0; i < msg.poses.size(); ++i) {
        geometry_msgs::msg::Pose pose = msg.poses[i];
        wind_data_.wind.poses.push_back(pose);
        RCLCPP_INFO(this->get_logger(), "Wind %zu - x: %f, y: %f, z: %f",
    i, wind_data_.wind.poses[i].position.x, wind_data_.wind.poses[i].position.y, wind_data_.wind.poses[i].position.z);

    }

    wind_data_.nb_wind = static_cast<int>(msg.poses.size());
    RCLCPP_INFO(this->get_logger(), "Total number of winds recorded: %d", wind_data_.nb_wind);
}

void TargetManagerNode::PathPlan(nav_msgs::msg::Path path)
{
    if (path.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received message without poses.");
        return;
    }

    path_data_.temp_.poses.clear();
    path_data_.temp_.poses.reserve(path.poses.size());
    path_data_.status.clear();
    path_data_.status.resize(path.poses.size(), false);
    path_data_.temp_.header.frame_id = "map";
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        geometry_msgs::msg::PoseStamped pose_stamped = path.poses[i];
        path_data_.temp_.poses.push_back(pose_stamped);
		RCLCPP_INFO(this->get_logger(), "Path %zu - x: %f, y: %f, z: %f",
            i, path_data_.temp_.poses[i].pose.position.x, path_data_.temp_.poses[i].pose.position.y, path_data_.temp_.poses[i].pose.position.z);

    }
    TargetManagerService_ = this->create_service<mission_manager::srv::TargetManagerServ>(
        "/mission/mission_goal",
        std::bind(&TargetManagerNode::ServerCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
}

void TargetManagerNode::ServerCallback(
    const std::shared_ptr<mission_manager::srv::TargetManagerServ::Request> request,
    const std::shared_ptr<mission_manager::srv::TargetManagerServ::Response> response)
{
   
    //verification du nombres d'eolienne checker
    path_sent = true;
    for (size_t i = 0; i < wind_data_.status.size(); ++i)
    {
       RCLCPP_INFO(this->get_logger(), "wind_data %d", static_cast<int>(wind_data_.status[i]));
        if (wind_data_.status[i] == false)
        {
            path_sent = false;
            break;
        }
    }
    path_sent = true; //a enlever
    //si c'est le dernier path
    if (path_sent == true)
    {   RCLCPP_INFO(this->get_logger(), "Test5");
        odometry_Subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
		    "/mission/odometry", 10,
		    std::bind(&TargetManagerNode::WindInspection, this, std::placeholders::_1));
        LastPath_Client_ = this->create_client<navigation::srv::PathLast>("/navigation/last_path");
        timer_inspec_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TargetManagerNode::launch_last, this));
        sleep(4);//de meme
            response->path = path_data_.temp_;
            response->targetcount.data = 1;
        //response->qr_orientation = this->wind_data_.pos_wind[static_cast<size_t>(wind_def)];
            response->qr_orientation = 1;
         RCLCPP_INFO(this->get_logger(), "Test6");

    }
    //RCLCPP_INFO(this->get_logger(), "Test0");
    //envoi du path simplement
    if (path_sent == false)
    {   
        if (path_data_.temp_.poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Path is empty");
            return;
        }
        //RCLCPP_INFO(this->get_logger(), "Test1");
        //ajout de la position du QR
        if (!request->cam.data.empty())
        {
            for (size_t i = 0; i < wind_data_.wind.poses.size(); ++i)
            {
                if (wind_data_.status[i] == false)
                {
                    this->wind_data_.qr[i] = request->cam;
                    this->wind_data_.pos_wind[i] = request->poswind;
                    this->wind_data_.status[i] = true;
                    break;
                }
            }
        }
        //RCLCPP_INFO(this->get_logger(), "Test2");
        //envoi du bon path
        for (size_t i = 0; i < path_data_.temp_.poses.size(); ++i)
        {//RCLCPP_INFO(this->get_logger(), "Test3");
            if (path_data_.status[i] == false)
            {
                path_data_.status[i] = true;
                response->path = path_data_.temp_;
                response->targetcount.data = static_cast<int32_t>(path_data_.temp_.poses.size());
                response->qr_orientation = 0.0;
            }
        }
        RCLCPP_INFO(this->get_logger(), "New path sent.");
    }
    /*
    for (size_t i = 0; i < wind_data_.status.size(); ++i)
    {
        if (!wind_data_.status[i])
        {
            wind_data_.status[i] = true;
        }
    }*/
/*
    auto nav_request = std::make_shared<navigation::srv::Path::Request>();
    auto future = this->TargetPath_Client_->async_send_request(
        nav_request,
        std::bind(&TargetManagerNode::service_response_callback, this, std::placeholders::_1)
    );*/
}

void TargetManagerNode::launch_last()
{

    RCLCPP_INFO(this->get_logger(), "Test4");
    if (!this->LastPath_Client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(this->get_logger(), "Service '/navigation/pathlast' not available");
        return;
    }
    timer_inspec_->cancel();
    if (wind_data_.wind.poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No wind data available.");
        return;
    }
     RCLCPP_INFO(this->get_logger(), "Test3");
    bool found_critical = false;
    for (size_t i = 0; i < wind_data_.wind.poses.size(); ++i)
    {
        if (i >= wind_data_.qr.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Index out of bounds for wind_data_.qr");
            return;
        }

        if (wind_data_.qr[i].data == "critical")
        {
            this->wind_def = static_cast<int32_t>(i);
            found_critical = true;
            break;
        }
    }
     RCLCPP_INFO(this->get_logger(), "Test2");
    /*if (!found_critical)
    {
        RCLCPP_WARN(this->get_logger(), "No critical wind found.");
        return;
    }*/

    RCLCPP_INFO(this->get_logger(), "Service is available. Sending request...");
     RCLCPP_INFO(this->get_logger(), "Test1");
    if (shipAdd)
    {
        auto request = std::make_shared<navigation::srv::PathLast::Request>();
        request->target_id = wind_def;
        request->ship_pos = ship;
        auto future = this->LastPath_Client_->async_send_request(
            request,
            std::bind(&TargetManagerNode::service_response_callback_last, this, std::placeholders::_1)
        );
    }
}

void TargetManagerNode::service_response_callback_last(
    rclcpp::Client<navigation::srv::PathLast>::SharedFuture future)
{
    RCLCPP_INFO(this->get_logger(), "Received response from service");

    auto response = future.get();

    if (response->path.poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Received empty poses from service. Retrying...");
        timer_inspec_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TargetManagerNode::launch_last, this));
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Processing received path");
        this->last_path = response->path;
    }
}

void TargetManagerNode::WindInspection(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Log the received odometry data for debugging
    RCLCPP_INFO(this->get_logger(), "Received odometry data - Position x: %f, y: %f, z: %f",
                msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    // Traitement des données d'odométrie, si nécessaire
    // Ici, on met à jour l'état de la variable `ship` avec les données reçues
    ship = *msg;
    shipAdd = true;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}