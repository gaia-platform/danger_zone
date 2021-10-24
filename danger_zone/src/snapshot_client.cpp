/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#include "snapshot_client.hpp"

#include "gaia/logger.hpp"

SnapshotClient::SnapshotClient()
{
}

/**
 * Connect to the ROS service
 *
 * @param[in] rclcpp::Node* caller
 * @param[in] std::string service_name
 * @return void
 * @throws
 * @exceptsafe yes
 */
void SnapshotClient::connect(rclcpp::Node* caller, std::string service_name)
{
    if (m_connected)
    {
        return;
    }

    m_caller_ptr = caller;
    m_service_name = service_name;

    m_snapshot_client = m_caller_ptr->create_client<TriggerSnapshot>(service_name);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service client created: %s", m_service_name.c_str());

    m_connected = true;
}

/**
 * Tell the ROS service to take a snapshot
 *
 * @param[in] int32_t start_sec
 * @param[in] uint32_t start_nsec
 * @param[in] int32_t end_sec
 * @param[in] uint32_t end_nsec
 * @param[in] std::string file_name
 * @param[in] std::vector<std::string>topic_names
 * @param[in] std::vector<std::string>topic_types
 * @return bool : success = true, failure = false;
 * @throws
 * @exceptsafe yes
 */
bool SnapshotClient::send_request(
    int32_t start_sec, uint32_t start_nsec, int32_t end_sec, uint32_t end_nsec,
    std::string file_name,
    std::vector<std::string> topic_names, std::vector<std::string> topic_types)
{
    if (!m_snapshot_client->wait_for_service(m_service_wait_time))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, exiting");
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service connected: %s", m_service_name.c_str());

    auto req = std::make_shared<TriggerSnapshot::Request>();

    builtin_interfaces::msg::Time start_time;
    builtin_interfaces::msg::Time end_time;

    start_time.set__sec(start_sec);
    start_time.set__nanosec(start_nsec);

    end_time.set__sec(end_sec);
    end_time.set__nanosec(end_nsec);

    req->set__start_time(start_time);
    req->set__stop_time(end_time);
    req->set__filename(file_name);

    auto topic_details = std::make_shared<std::vector<rosbag2_snapshot_msgs::msg::TopicDetails>>();
    for (std::size_t index = 0; index < topic_names.size(); ++index)
    {
        auto topic_detail = std::make_shared<rosbag2_snapshot_msgs::msg::TopicDetails>();

        topic_detail->set__name(topic_names[index]);
        topic_detail->set__type(topic_types[index]);

        topic_details->push_back(*topic_detail);
    }
    req->set__topics(*topic_details);

    std::string top;
    for (const auto& topic : req->topics)
    {
        top += topic.name + " [" + topic.type + "],";
    }

    gaia_log::app().info("\nTriggerSnapshot::Request \nI'll tr"
                         "start_time_sec: {} \n"
                         "start_time_nsec: {} \n"
                         "end_time_sec: {} \n"
                         "end_time_nsec: {} \n"
                         "filename: {} \n"
                         "topics: {}",
                         req->start_time.sec, req->start_time.nanosec, req->stop_time.sec, req->stop_time.nanosec, req->filename, top);



    auto result = m_snapshot_client->async_send_request(req);

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Service request sent: %s", m_service_name.c_str());

    /* TODO * enable?

    if (rclcpp::spin_until_future_complete(m_snapshot_client, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: %s : %s",
        result.get()->success?"success":"failue", result.get()->message.c_str());
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }*/

    return true;
}
