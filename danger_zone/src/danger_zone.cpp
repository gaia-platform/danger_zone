/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#include "danger_zone.hpp"

#include <functional>
#include <memory>

#include <danger_zone_msgs/msg/obstacle_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "gaia/logger.hpp"
#include "gaia/rules/rules.hpp"
#include "gaia/system.hpp"

#include "gaia_danger_zone.h"
#include "gaia_db.hpp"
#include "snapshot_client.hpp"
#include "zones.hpp"

using namespace gaia::danger_zone;

struct danger_zone_obstacles_t : public obstacles_t
{
    std::vector<danger_zone_msgs::msg::Obstacle> obstacles_vector;

    // obstacles_t interface implementation.

    void add(
        std::string type_name, uint8_t roi, uint8_t direction,
        double pos_x, double pos_y, double pos_z,
        double size_x, double size_y, double size_z,
        double orient_x, double orient_y, double orient_z, double orient_w) override
    {
        obstacles_vector.push_back(*(build_obstacle_message(
            type_name, roi, direction, pos_x, pos_y, pos_z,
            size_x, size_y, size_z, orient_x, orient_y, orient_z, orient_w)));
    }

    // Static helpers for building obstacle-related messages

    static danger_zone_msgs::msg::ObstacleArray::UniquePtr build_obstacle_array_message(
        std::vector<danger_zone_msgs::msg::Obstacle> obstacles,
        std::string frame_id, int32_t seconds, uint32_t nano_seconds)
    {
        danger_zone_msgs::msg::ObstacleArray::UniquePtr obstacle_array(new danger_zone_msgs::msg::ObstacleArray);

        obstacle_array->header = std_msgs::msg::Header();
        obstacle_array->header.frame_id = frame_id;
        obstacle_array->header.stamp.sec = seconds;
        obstacle_array->header.stamp.nanosec = nano_seconds;

        for (const auto& obstacle : obstacles)
        {
            obstacle_array->obstacles.push_back(obstacle);
        }

        return obstacle_array;
    }

    static danger_zone_msgs::msg::Obstacle::UniquePtr build_obstacle_message(
        std::string type_name, uint8_t roi, uint8_t direction,
        double pos_x, double pos_y, double pos_z,
        double size_x, double size_y, double size_z,
        double orient_x, double orient_y, double orient_z, double orient_w)
    {
        danger_zone_msgs::msg::Obstacle::UniquePtr obstacle(new danger_zone_msgs::msg::Obstacle);

        geometry_msgs::msg::Point::UniquePtr point(new geometry_msgs::msg::Point);
        point->x = pos_x;
        point->y = pos_y;
        point->z = pos_z;

        geometry_msgs::msg::Vector3::UniquePtr size(new geometry_msgs::msg::Vector3);
        size->x = size_x;
        size->y = size_y;
        size->z = size_z;

        geometry_msgs::msg::Quaternion::UniquePtr orient(new geometry_msgs::msg::Quaternion);
        orient->x = orient_x;
        orient->y = orient_y;
        orient->z = orient_z;
        orient->w = orient_w;

        geometry_msgs::msg::Pose::UniquePtr pose(new geometry_msgs::msg::Pose);
        pose->position = *point;
        pose->orientation = *orient;

        vision_msgs::msg::BoundingBox3D::UniquePtr bbox(new vision_msgs::msg::BoundingBox3D);
        bbox->center = *pose;
        bbox->size = *size;

        obstacle->type = type_name;
        obstacle->roi = zones_t::convert_zone_id_to_simulation_id(roi);
        obstacle->direction = direction;
        obstacle->bounds = *bbox;

        return obstacle;
    }
};

std::shared_ptr<obstacles_t> obstacles_t::new_instance()
{
    return std::make_shared<danger_zone_obstacles_t>();
}

class subscriber_node_t : public rclcpp::Node, danger_zone_t
{
public:
    subscriber_node_t()
        : Node("danger_zone_ros")
    {
        // TODO: make this modern.
        danger_zone_ptr = static_cast<danger_zone_t*>(this);

        this->create_subscription<vision_msgs::msg::Detection3DArray>(
            m_detected_topic_name, 10, std::bind(&subscriber_node_t::detection3d_callback, this, _1));

        m_obstacles_pub = this->create_publisher<danger_zone_msgs::msg::ObstacleArray>(
            m_obstacles_topic_name, 1);

        m_snapshot_client = std::make_shared<SnapshotClient>();
        m_snapshot_client->connect(this, m_snapshot_service_name);
    }

    // danger_zone_t interface implementation.

    void send_obstacle_array_message(
        std::shared_ptr<obstacles_t> obstacles,
        std::string frame_id, int32_t seconds, uint32_t nano_seconds) override
    {
        std::shared_ptr<danger_zone_obstacles_t> danger_zone_obstacles
            = std::dynamic_pointer_cast<danger_zone_obstacles_t>(obstacles);

        m_obstacles_pub->publish(danger_zone_obstacles_t::build_obstacle_array_message(
            danger_zone_obstacles->obstacles_vector, frame_id, seconds, nano_seconds));
    }

    void trigger_log(
        int32_t base_seconds, uint32_t base_nano_seconds,
        int32_t seconds_past, int32_t seconds_forward,
        std::string file_name,
        std::vector<std::string> topic_names, std::vector<std::string> topic_types) override
    {
        int32_t start_seconds = base_seconds - seconds_past;
        int32_t end_seconds = base_seconds + seconds_forward;
        m_snapshot_client->send_request(
            start_seconds, base_nano_seconds,
            end_seconds, base_nano_seconds,
            file_name,
            topic_names, topic_types);
    }

private:
    void detection3d_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        gaia::db::begin_transaction();

        // Create a detection record to reference all detected objects.
        auto db_detection_id = gaia::danger_zone::detection_t::insert_row(
            msg->header.frame_id.c_str(), msg->header.stamp.sec, msg->header.stamp.nanosec, false);
        auto db_detection = gaia::danger_zone::detection_t::get(db_detection_id);

        for (const vision_msgs::msg::Detection3D& detection : msg->detections)
        {
            vision_msgs::msg::ObjectHypothesisWithPose max_hyp;

            max_hyp.hypothesis.class_id = "";
            max_hyp.hypothesis.score = 0.0;

            for (const auto& result : detection.results)
            {
                if (result.hypothesis.score > max_hyp.hypothesis.score)
                {
                    max_hyp = result;
                }
            }

            if (max_hyp.hypothesis.class_id.empty())
            {
                gaia_log::app().warn("Detected object with no class_id!");
                continue;
            }

            object_t object = get_object(detection.id.c_str(), max_hyp.hypothesis.class_id.c_str());

            // Note: detection.id.c_str() is non-unique ATM, it does not seem an ID either.
            auto db_detected_object_id = gaia::danger_zone::d_object_t::insert_row(
                object.id(), max_hyp.hypothesis.score, 0, 0,
                detection.bbox.center.position.x, detection.bbox.center.position.y,
                detection.bbox.center.position.z,
                detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z,
                max_hyp.pose.pose.orientation.x, max_hyp.pose.pose.orientation.y,
                max_hyp.pose.pose.orientation.z, max_hyp.pose.pose.orientation.w,
                msg->header.stamp.sec, msg->header.stamp.nanosec,
                zones_t::c_no_zone);
            auto db_detected_object = gaia::danger_zone::d_object_t::get(db_detected_object_id);

            // Add the detected object to our detection record.
            db_detection.d_objects().connect(db_detected_object);
        }

        gaia::db::commit_transaction();
    }

private:
    const std::string m_detected_topic_name = "/komatsu/detections";
    const std::string m_obstacles_topic_name = "/komatsu/obstacles";
    // Name found in snapshotter.cpp.
    const std::string m_snapshot_service_name = "trigger_snapshot";

    rclcpp::Publisher<danger_zone_msgs::msg::ObstacleArray>::SharedPtr m_obstacles_pub;
    std::shared_ptr<SnapshotClient> m_snapshot_client;
};

int main(int argc, char* argv[])
{
    gaia::system::initialize();

    gaia::db::begin_transaction();
    clean_db();
    gaia::db::commit_transaction();

    gaia::db::begin_transaction();
    initialize_zones();
    initialize_object_classes();
    gaia::db::commit_transaction();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<subscriber_node_t>());
    rclcpp::shutdown();
    return 0;
}
