#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

struct PersistentMarker
{
    visualization_msgs::msg::Marker marker;
    rclcpp::Time last_seen;
    int detection_count;
    double confidence_score;

    PersistentMarker() : detection_count(1), confidence_score(1.0) {}
};

class CameraToMapTransformer : public rclcpp::Node
{
public:
    CameraToMapTransformer()
        : Node("camera_to_map_transformer"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          next_unique_id_(0)
    {
        // parameters
        this->declare_parameter("merge_distance_threshold", 0.2); // 10cm
        this->declare_parameter("max_detection_distance", 10.0);  // 10m
        this->declare_parameter("marker_timeout", 30.0);          // 30 seconds
        this->declare_parameter("min_detections_for_persistence", 10);
        this->declare_parameter("confidence_decay_rate", 0.95);
        this->declare_parameter("min_confidence_threshold", 0.1);

        merge_threshold_ = this->get_parameter("merge_distance_threshold").as_double();
        max_distance_ = this->get_parameter("max_detection_distance").as_double();
        marker_timeout_ = this->get_parameter("marker_timeout").as_double();
        min_detections_ = this->get_parameter("min_detections_for_persistence").as_int();
        confidence_decay_ = this->get_parameter("confidence_decay_rate").as_double();
        min_confidence_ = this->get_parameter("min_confidence_threshold").as_double();

        // Subscriber: expects markers stamped in "camera_link"
        raw_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/detected_objects/raw",
            10,
            std::bind(&CameraToMapTransformer::rawCallback, this, std::placeholders::_1));

        // Publisher: will publish markers in "map"
        map_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/detected_objects/markers", 5);

        cleanup_timer_ = this->create_wall_timer(
            1s, std::bind(&CameraToMapTransformer::cleanupCallback, this));

        RCLCPP_INFO(this->get_logger(), "camera_to_map_transformer node started");
        RCLCPP_INFO(this->get_logger(), "Merge threshold: %.2fm, Max distance: %.2fm, Timeout: %.1fs",
                    merge_threshold_, max_distance_, marker_timeout_);
    }

private:
    double calculateDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) +
                         std::pow(p1.y - p2.y, 2) +
                         std::pow(p1.z - p2.z, 2));
    }

    double calculateDistanceFromOrigin(const geometry_msgs::msg::Point &p)
    {
        return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    bool isValidDetection(const geometry_msgs::msg::Point &point)
    {
        double distance = calculateDistanceFromOrigin(point);

        // Filter out invalid detections (distance = 0 or very close to 0)
        if (distance < 0.01) // 1cm minimum
        {
            return false;
        }

        // Filter out detections that are too far
        if (distance > max_distance_)
        {
            return false;
        }

        // Check for NaN or infinite values
        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
            std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z))
        {
            return false;
        }

        return true;
    }

    std::string findNearbyMarker(const geometry_msgs::msg::Point &new_point, const std::string &marker_type)
    {
        for (const auto &[key, persistent_marker] : persistent_markers_)
        {
            // Only compare with markers of the same type/namespace
            if (persistent_marker.marker.ns != marker_type)
                continue;

            double distance = calculateDistance(new_point, persistent_marker.marker.pose.position);
            if (distance <= merge_threshold_)
            {
                return key;
            }
        }
        return "";
    }

    void updateMarkerPosition(const std::string &key, const geometry_msgs::msg::Pose &new_pose,
                              const rclcpp::Time &timestamp)
    {
        auto &persistent_marker = persistent_markers_[key];

        // Weighted average based on detection count (gives more weight to established markers)
        double weight = 1.0 / (persistent_marker.detection_count + 1);

        persistent_marker.marker.pose.position.x =
            (1.0 - weight) * persistent_marker.marker.pose.position.x + weight * new_pose.position.x;
        persistent_marker.marker.pose.position.y =
            (1.0 - weight) * persistent_marker.marker.pose.position.y + weight * new_pose.position.y;
        persistent_marker.marker.pose.position.z =
            (1.0 - weight) * persistent_marker.marker.pose.position.z + weight * new_pose.position.z;

        persistent_marker.marker.pose.orientation = new_pose.orientation;

        persistent_marker.last_seen = timestamp;
        persistent_marker.detection_count++;
        persistent_marker.confidence_score = std::min(1.0, persistent_marker.confidence_score + 0.1);

        // Update marker timestamp
        persistent_marker.marker.header.stamp = timestamp;
    }

    void addNewMarker(const visualization_msgs::msg::Marker &original_marker,
                      const geometry_msgs::msg::Pose &map_pose,
                      const rclcpp::Time &timestamp)
    {
        std::string key = original_marker.ns + "_" + std::to_string(next_unique_id_++);

        PersistentMarker persistent_marker;
        persistent_marker.marker = original_marker;
        persistent_marker.marker.header.frame_id = "map";
        persistent_marker.marker.header.stamp = timestamp;
        persistent_marker.marker.ns = original_marker.ns;
        persistent_marker.marker.id = next_unique_id_ - 1;
        persistent_marker.marker.pose = map_pose;
        persistent_marker.last_seen = timestamp;
        persistent_marker.detection_count = 1;
        persistent_marker.confidence_score = 0.5; // Start with medium confidence

        // Set marker to persist until explicitly deleted
        persistent_marker.marker.lifetime = rclcpp::Duration(0, 0);

        persistent_markers_[key] = persistent_marker;
    }

    void cleanupCallback()
    {
        rclcpp::Time now = this->get_clock()->now();
        std::vector<std::string> keys_to_remove;

        for (auto &[key, persistent_marker] : persistent_markers_)
        {
            // Decay confidence over time
            persistent_marker.confidence_score *= confidence_decay_;

            // Check for timeout or low confidence
            double time_since_last_seen = (now - persistent_marker.last_seen).seconds();

            if (persistent_marker.detection_count < 50 && (time_since_last_seen > marker_timeout_ ||
                                                           persistent_marker.confidence_score < min_confidence_))
            {
                keys_to_remove.push_back(key);
            }
        }

        // Remove expired markers
        for (const auto &key : keys_to_remove)
        {
            persistent_markers_.erase(key);
        }

        // Publish current marker array
        publishPersistentMarkers();
    }

    void publishPersistentMarkers()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        for (const auto &[key, persistent_marker] : persistent_markers_)
        {
            // Only publish markers that have been seen multiple times (more reliable)
            if (persistent_marker.detection_count >= min_detections_)
            {
                auto marker = persistent_marker.marker;

                // Adjust marker transparency based on confidence
                marker.color.a = std::max(0.3, persistent_marker.confidence_score);

                // Add text showing detection count
                marker.text = "Count: " + std::to_string(persistent_marker.detection_count) +
                              " Conf: " + std::to_string(static_cast<int>(persistent_marker.confidence_score * 100)) + "%";

                marker_array.markers.push_back(marker);
            }
        }

        // Add deletion markers for any removed markers
        static std::vector<int> previous_ids;
        std::vector<int> current_ids;

        for (const auto &marker : marker_array.markers)
        {
            current_ids.push_back(marker.id);
        }

        for (int prev_id : previous_ids)
        {
            if (std::find(current_ids.begin(), current_ids.end(), prev_id) == current_ids.end())
            {
                visualization_msgs::msg::Marker delete_marker;
                delete_marker.header.frame_id = "map";
                delete_marker.header.stamp = this->get_clock()->now();
                delete_marker.ns = "persistent";
                delete_marker.id = prev_id;
                delete_marker.action = visualization_msgs::msg::Marker::DELETE;
                marker_array.markers.push_back(delete_marker);
            }
        }

        previous_ids = current_ids;

        if (!marker_array.markers.empty())
        {
            map_pub_->publish(marker_array);
            RCLCPP_INFO(this->get_logger(), "Published %zu persistent markers", marker_array.markers.size());
        }
    }

    void rawCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        rclcpp::Time callback_time = this->get_clock()->now();

        for (const auto &m : msg->markers)
        {
            // Skip invalid markers
            if (!isValidDetection(m.pose.position))
            {
                RCLCPP_INFO(this->get_logger(), "Skipping invalid detection at distance %.2f",
                            calculateDistanceFromOrigin(m.pose.position));
                continue;
            }

            // build a PointStamped from the raw marker s pose.position
            geometry_msgs::msg::PoseStamped pose_cam;
            pose_cam.header = m.header; // frame_id="camera_link", stamp = detection time
            pose_cam.pose = m.pose;

            // transform from "camera_link" -> "map"
            try
            {
                if (tf_buffer_.canTransform(
                        "map",
                        pose_cam.header.frame_id,
                        pose_cam.header.stamp,
                        rclcpp::Duration(0, 100000000)))
                {
                    geometry_msgs::msg::PoseStamped pose_map;
                    tf_buffer_.transform(pose_cam, pose_map, "map");

                    std::string nearby_key = findNearbyMarker(pose_map.pose.position, m.ns);

                    if (!nearby_key.empty())
                    {
                        // update existing marker
                        updateMarkerPosition(nearby_key, pose_map.pose, callback_time);
                    }
                    else
                    {
                        // add new marker
                        addNewMarker(m, pose_map.pose, callback_time);
                    }
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        2000,
                        "TF not yet available for %s at stamp %u.%u",
                        pose_cam.header.frame_id.c_str(),
                        pose_cam.header.stamp.sec,
                        pose_cam.header.stamp.nanosec);
                }
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "TF Exception: %s", ex.what());
            }
        }
    }

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr raw_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr cleanup_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::unordered_map<std::string, PersistentMarker> persistent_markers_;
    int next_unique_id_;

    // Parameters
    double merge_threshold_;
    double max_distance_;
    double marker_timeout_;
    int min_detections_;
    double confidence_decay_;
    double min_confidence_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraToMapTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
