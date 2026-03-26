#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/solution.hpp>

namespace {

std::string solutionStatusName(libgnss::SolutionStatus status) {
    switch (status) {
        case libgnss::SolutionStatus::NONE:
            return "NONE";
        case libgnss::SolutionStatus::SPP:
            return "SPP";
        case libgnss::SolutionStatus::DGPS:
            return "DGPS";
        case libgnss::SolutionStatus::FLOAT:
            return "FLOAT";
        case libgnss::SolutionStatus::FIXED:
            return "FIXED";
        case libgnss::SolutionStatus::PPP_FLOAT:
            return "PPP_FLOAT";
        case libgnss::SolutionStatus::PPP_FIXED:
            return "PPP_FIXED";
        default:
            return "UNKNOWN";
    }
}

class GnssSolutionNode final : public rclcpp::Node {
public:
    GnssSolutionNode()
        : rclcpp::Node("gnss_solution_node") {
        declare_parameter<std::string>("solution_file", "");
        declare_parameter<std::string>("fix_topic", "/gnss/fix");
        declare_parameter<std::string>("pose_topic", "/gnss/ecef_pose");
        declare_parameter<std::string>("path_topic", "/gnss/path");
        declare_parameter<std::string>("status_topic", "/gnss/status");
        declare_parameter<std::string>("satellite_count_topic", "/gnss/num_satellites");
        declare_parameter<std::string>("frame_id", "earth");
        declare_parameter<int>("publish_period_ms", 200);
        declare_parameter<int>("max_messages", 0);
        declare_parameter<bool>("loop", false);
        declare_parameter<bool>("publish_status", true);
        declare_parameter<bool>("publish_path", true);

        solution_file_ = get_parameter("solution_file").as_string();
        if (solution_file_.empty()) {
            throw std::runtime_error("parameter `solution_file` is required");
        }

        if (!solution_.loadFromFile(solution_file_)) {
            throw std::runtime_error("failed to load solution file: " + solution_file_);
        }
        if (solution_.isEmpty()) {
            throw std::runtime_error("solution file contains no records: " + solution_file_);
        }

        fix_topic_ = get_parameter("fix_topic").as_string();
        pose_topic_ = get_parameter("pose_topic").as_string();
        path_topic_ = get_parameter("path_topic").as_string();
        status_topic_ = get_parameter("status_topic").as_string();
        satellite_count_topic_ = get_parameter("satellite_count_topic").as_string();
        frame_id_ = get_parameter("frame_id").as_string();
        publish_period_ms_ = std::max<int>(1, static_cast<int>(get_parameter("publish_period_ms").as_int()));
        max_messages_ = std::max<int>(0, static_cast<int>(get_parameter("max_messages").as_int()));
        loop_ = get_parameter("loop").as_bool();
        publish_status_ = get_parameter("publish_status").as_bool();
        publish_path_ = get_parameter("publish_path").as_bool();

        fix_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(fix_topic_, 10);
        pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
        if (publish_path_) {
            path_publisher_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
            path_message_.header.frame_id = frame_id_;
        }
        if (publish_status_) {
            status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
            satellite_count_publisher_ = create_publisher<std_msgs::msg::Int32>(satellite_count_topic_, 10);
        }

        RCLCPP_INFO(
            get_logger(),
            "Loaded %zu solutions from %s (fix_topic=%s pose_topic=%s path_topic=%s status_topic=%s sats_topic=%s)",
            solution_.size(),
            solution_file_.c_str(),
            fix_topic_.c_str(),
            pose_topic_.c_str(),
            path_topic_.c_str(),
            status_topic_.c_str(),
            satellite_count_topic_.c_str());

        timer_ = create_wall_timer(
            std::chrono::milliseconds(publish_period_ms_),
            std::bind(&GnssSolutionNode::publishNext, this));
    }

    ~GnssSolutionNode() override {
        RCLCPP_INFO(
            get_logger(),
            "published_messages=%zu published_fixed=%zu published_float=%zu published_ppp_fixed=%zu published_ppp_float=%zu",
            published_messages_,
            fixed_messages_,
            float_messages_,
            ppp_fixed_messages_,
            ppp_float_messages_);
        if (publish_path_) {
            RCLCPP_INFO(
                get_logger(),
                "published_path_points=%zu",
                path_message_.poses.size());
        }
    }

private:
    void publishNext() {
        if (solution_.isEmpty()) {
            rclcpp::shutdown();
            return;
        }

        if (next_index_ >= solution_.size()) {
            if (loop_) {
                next_index_ = 0;
            } else {
                RCLCPP_INFO(get_logger(), "Reached end of solution file");
                rclcpp::shutdown();
                return;
            }
        }

        const auto& solution = solution_.solutions[next_index_++];
        publishFix(solution);
        publishPose(solution);
        publishStatus(solution);
        ++published_messages_;
        updateStatusCounters(solution.status);

        if (max_messages_ > 0 && static_cast<int>(published_messages_) >= max_messages_) {
            RCLCPP_INFO(get_logger(), "Reached max_messages=%d", max_messages_);
            rclcpp::shutdown();
        }
    }

    void publishFix(const libgnss::PositionSolution& solution) {
        sensor_msgs::msg::NavSatFix msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id_;
        msg.latitude = solution.position_geodetic.latitude * 180.0 / M_PI;
        msg.longitude = solution.position_geodetic.longitude * 180.0 / M_PI;
        msg.altitude = solution.position_geodetic.height;
        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        msg.status.status = solution.isValid()
            ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
            : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        msg.position_covariance[0] = solution.position_covariance(0, 0);
        msg.position_covariance[4] = solution.position_covariance(1, 1);
        msg.position_covariance[8] = solution.position_covariance(2, 2);
        fix_publisher_->publish(msg);
    }

    void publishPose(const libgnss::PositionSolution& solution) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id_;
        msg.pose.position.x = solution.position_ecef.x();
        msg.pose.position.y = solution.position_ecef.y();
        msg.pose.position.z = solution.position_ecef.z();
        msg.pose.orientation.w = 1.0;
        pose_publisher_->publish(msg);
        if (publish_path_ && path_publisher_ != nullptr) {
            path_message_.header.stamp = msg.header.stamp;
            path_message_.poses.push_back(msg);
            path_publisher_->publish(path_message_);
        }
    }

    void publishStatus(const libgnss::PositionSolution& solution) {
        if (!publish_status_ || status_publisher_ == nullptr || satellite_count_publisher_ == nullptr) {
            return;
        }
        std_msgs::msg::String status_msg;
        status_msg.data = solutionStatusName(solution.status);
        status_publisher_->publish(status_msg);

        std_msgs::msg::Int32 satellite_msg;
        satellite_msg.data = solution.num_satellites;
        satellite_count_publisher_->publish(satellite_msg);
    }

    void updateStatusCounters(libgnss::SolutionStatus status) {
        switch (status) {
            case libgnss::SolutionStatus::FIXED:
                ++fixed_messages_;
                break;
            case libgnss::SolutionStatus::FLOAT:
                ++float_messages_;
                break;
            case libgnss::SolutionStatus::PPP_FIXED:
                ++ppp_fixed_messages_;
                break;
            case libgnss::SolutionStatus::PPP_FLOAT:
                ++ppp_float_messages_;
                break;
            default:
                break;
        }
    }

    libgnss::Solution solution_;
    std::string solution_file_;
    std::string fix_topic_;
    std::string pose_topic_;
    std::string path_topic_;
    std::string status_topic_;
    std::string satellite_count_topic_;
    std::string frame_id_;
    int publish_period_ms_ = 200;
    int max_messages_ = 0;
    bool loop_ = false;
    bool publish_status_ = true;
    bool publish_path_ = true;
    size_t next_index_ = 0;
    size_t published_messages_ = 0;
    size_t fixed_messages_ = 0;
    size_t float_messages_ = 0;
    size_t ppp_fixed_messages_ = 0;
    size_t ppp_float_messages_ = 0;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr satellite_count_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path path_message_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<GnssSolutionNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("gnss_solution_node"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
