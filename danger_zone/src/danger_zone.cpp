////////////////////////////////////////////////////
// Copyright (c) Gaia Platform Authors
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include "danger_zone.hpp"

#include <functional>
#include <memory>

#include <danger_zone_msgs/msg/obstacle_array.hpp>
#include <danger_zone_msgs/msg/snapshot_triggered.hpp>
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
        std::string frame_id, int32_t seconds, uint32_t nanoseconds)
    {
        danger_zone_msgs::msg::ObstacleArray::UniquePtr obstacle_array(
            new danger_zone_msgs::msg::ObstacleArray);

        obstacle_array->header = std_msgs::msg::Header();
        obstacle_array->header.frame_id = frame_id;
        obstacle_array->header.stamp.sec = seconds;
        obstacle_array->header.stamp.nanosec = nanoseconds;

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

        set_ego_shape();

        m_detection3d_subscription = this->create_subscription<vision_msgs::msg::Detection3DArray>(
            m_detected_topic_name, 10, std::bind(&subscriber_node_t::detection3d_callback, this, _1));

        m_obstacles_pub = this->create_publisher<danger_zone_msgs::msg::ObstacleArray>(
            m_obstacles_topic_name, 1);

        m_triggered_pub = this->create_publisher<danger_zone_msgs::msg::SnapshotTriggered>(
            m_triggered_topic_name, 1);

        m_snapshot_client = std::make_shared<SnapshotClient>();
        m_snapshot_client->connect(this, m_snapshot_service_name);
    }

    // danger_zone_t interface implementation.

    std::vector<zones_t::point_3d> get_ego_shape() override
    {
        return m_ego_shape;
    }

    void get_current_time(int32_t& seconds, uint32_t& nanoseconds) override
    {
        auto current_time = get_clock()->now();
        seconds = current_time.seconds();
        nanoseconds = current_time.nanoseconds();
        gaia_log::app().info("Current time is: ({}, {}).", seconds, nanoseconds);
    }

    danger_zone_msgs::msg::SnapshotTriggered::UniquePtr build_triggered_message(
        int32_t start_sec, uint32_t start_nsec, int32_t end_sec, uint32_t end_nsec,
        std::string file_name,
        const std::vector<std::string> &topic_names, const std::vector<std::string> &topic_types )
    {
        auto trig_msg =
            std::make_unique<danger_zone_msgs::msg::SnapshotTriggered>();

        trig_msg->header = std_msgs::msg::Header();
        trig_msg->header.stamp = now();

        builtin_interfaces::msg::Time start_time;
        builtin_interfaces::msg::Time end_time;

        start_time.sec = start_sec;
        start_time.nanosec = start_nsec;

        end_time.set__sec(end_sec);
        end_time.set__nanosec(end_nsec);

        trig_msg->set__start_time(start_time);
        trig_msg->set__stop_time(end_time);
        trig_msg->set__filename(file_name);

        auto topic_details = std::vector<danger_zone_msgs::msg::TriggeredTopicDetails>();

        for (std::size_t index = 0; index < topic_names.size(); ++index)
        {
            auto topic_detail = std::make_shared<danger_zone_msgs::msg::TriggeredTopicDetails>();

            topic_detail->set__name(topic_names[index]);
            topic_detail->set__type(topic_types[index]);

            topic_details.push_back(*topic_detail);
        }
        trig_msg->set__topics(topic_details);

        return trig_msg;
    }

    void send_obstacle_array_message(
        std::shared_ptr<obstacles_t> obstacles,
        std::string frame_id, int32_t seconds, uint32_t nanoseconds) override
    {
        std::shared_ptr<danger_zone_obstacles_t> danger_zone_obstacles
            = std::dynamic_pointer_cast<danger_zone_obstacles_t>(obstacles);

        m_obstacles_pub->publish(danger_zone_obstacles_t::build_obstacle_array_message(
            danger_zone_obstacles->obstacles_vector, frame_id, seconds, nanoseconds));
    }

    void trigger_log(
        int32_t begin_seconds, uint32_t begin_nanoseconds,
        int32_t end_seconds, uint32_t end_nanoseconds,
        std::string file_name,
        std::vector<std::string> topic_names, std::vector<std::string> topic_types) override
    {
        m_triggered_pub->publish(build_triggered_message(
            begin_seconds, begin_nanoseconds,
            end_seconds, end_nanoseconds,
            file_name, topic_names, topic_types));

        m_snapshot_client->send_request(
            begin_seconds, begin_nanoseconds,
            end_seconds, end_nanoseconds,
            file_name, topic_names, topic_types);
    }

private:
    void detection3d_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        gaia::db::begin_transaction();

        // The time coming from the simulation is "midnight" based, while ROS is epoch based.
        // We need to use the ROS time to correctly trigger the snapshot which messages are
        // recorded according to ROS time.
        // For this reason we override the simulation time with the current ROS time. This is
        // a workaround that hopefully can be removed once the simulation time is fixed.
        rclcpp::Time now = get_clock()->now();

        // Create a detection record to reference all detected objects.
        auto db_detection_id = gaia::danger_zone::detection_t::insert_row(
            msg->header.frame_id.c_str(), now.seconds(), now.nanoseconds(), false);
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
                object.id(), max_hyp.hypothesis.score,
                detection.bbox.center.position.x, detection.bbox.center.position.y,
                detection.bbox.center.position.z,
                detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z,
                max_hyp.pose.pose.orientation.x, max_hyp.pose.pose.orientation.y,
                max_hyp.pose.pose.orientation.z, max_hyp.pose.pose.orientation.w,
                now.seconds(), now.nanoseconds(),
                zones_t::c_no_zone);
            auto db_detected_object = gaia::danger_zone::d_object_t::get(db_detected_object_id);

            // Add the detected object to our detection record.
            db_detection.d_objects().connect(db_detected_object);
        }

        gaia::db::commit_transaction();
    }

    void set_ego_shape()
    {
        // For demo only. Create an ego shape with eight vertices.
        // In production you would set these in config and read them in here.
        // If the shape of the ego can change, for example if it has
        // an articulated arm, then the shape would be more complex and
        // would have to be updated at runtime.

        m_ego_shape.push_back(zones_t::point_3d(4,2,4));
        m_ego_shape.push_back(zones_t::point_3d(4,2,4));
        m_ego_shape.push_back(zones_t::point_3d(4,-2,4));
        m_ego_shape.push_back(zones_t::point_3d(4,-2,4));
        m_ego_shape.push_back(zones_t::point_3d(-4,2,0));
        m_ego_shape.push_back(zones_t::point_3d(-4,-2,0));
        m_ego_shape.push_back(zones_t::point_3d(-4,2,0));
        m_ego_shape.push_back(zones_t::point_3d(-4,-2,0));
    }

private:
    const std::string m_detected_topic_name = "detections";
    const std::string m_obstacles_topic_name = "obstacles";
    const std::string m_triggered_topic_name = "triggered";
    // Name found in snapshotter.cpp.
    const std::string m_snapshot_service_name = "trigger_snapshot";

    [[maybe_unused]] rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr m_detection3d_subscription;
    rclcpp::Publisher<danger_zone_msgs::msg::ObstacleArray>::SharedPtr m_obstacles_pub;
    rclcpp::Publisher<danger_zone_msgs::msg::SnapshotTriggered>::SharedPtr m_triggered_pub;
    std::shared_ptr<SnapshotClient> m_snapshot_client;
};

int main(int argc, char* argv[])
{
    gaia::system::initialize();

    gaia_log::app().info("Cleaning database...");
    gaia::db::begin_transaction();
    clean_db();
    gaia::db::commit_transaction();
    gaia_log::app().info("Database clean complete!");

    gaia_log::app().info("Initializing database...");
    gaia::db::begin_transaction();
    initialize_zones();
    initialize_object_classes();
    initialize_logging_state();
    gaia::db::commit_transaction();
    gaia_log::app().info("Database initialization complete!");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<subscriber_node_t>());
    rclcpp::shutdown();
    gaia::system::shutdown();
    return 0;
}
