/**
 * @file gnss_raw_driver_node.cpp
 * @brief ROS 2 driver node for GNSS receivers (UBX/SBF binary protocols).
 *
 * Reads from serial port, decodes UBX or SBF messages, and publishes:
 *   - /gnss/fix            (sensor_msgs/NavSatFix)
 *   - /gnss/raw            (gnss_raw_driver/GnssRawEpoch)
 *   - /gnss/raw_binary     (std_msgs/UInt8MultiArray)  for rosbag recording
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "gnss_raw_driver/msg/gnss_raw_epoch.hpp"
#include "gnss_raw_driver/msg/gnss_raw_observation.hpp"

#include "libgnss++/io/ubx.hpp"
#include "libgnss++/io/sbf.hpp"
#include "libgnss++/io/serial_port.hpp"
#include "libgnss++/io/protocol_detector.hpp"

using namespace libgnss;
using namespace libgnss::io;

class GnssRawDriverNode : public rclcpp::Node {
public:
    GnssRawDriverNode() : Node("gnss_raw_driver") {
        declare_parameter("device", "/dev/ttyUSB0");
        declare_parameter("baud_rate", 115200);
        declare_parameter("protocol", "auto");
        declare_parameter("frame_id", "gnss");
        declare_parameter("publish_raw_binary", true);

        device_ = get_parameter("device").as_string();
        baud_rate_ = get_parameter("baud_rate").as_int();
        protocol_str_ = get_parameter("protocol").as_string();
        frame_id_ = get_parameter("frame_id").as_string();
        publish_raw_binary_ = get_parameter("publish_raw_binary").as_bool();

        fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("gnss/fix", 10);
        raw_pub_ = create_publisher<gnss_raw_driver::msg::GnssRawEpoch>("gnss/raw", 10);
        if (publish_raw_binary_) {
            binary_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(
                "gnss/raw_binary", 100);
        }

        if (protocol_str_ == "ubx") use_ubx_ = true;
        else if (protocol_str_ == "sbf") use_sbf_ = true;

        SerialPort::Config cfg;
        cfg.device = device_;
        cfg.baud_rate = static_cast<uint32_t>(baud_rate_);

        if (!serial_.open(cfg)) {
            RCLCPP_ERROR(get_logger(), "Failed to open %s at %d baud",
                         device_.c_str(), baud_rate_);
            return;
        }
        RCLCPP_INFO(get_logger(), "Opened %s @ %d baud, protocol=%s",
                     device_.c_str(), baud_rate_, protocol_str_.c_str());

        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&GnssRawDriverNode::readCallback, this));
    }

private:
    void readCallback() {
        uint8_t buf[4096];
        ssize_t n = serial_.read(buf, sizeof(buf));
        if (n <= 0) return;

        size_t len = static_cast<size_t>(n);

        if (publish_raw_binary_ && binary_pub_) {
            auto msg = std::make_unique<std_msgs::msg::UInt8MultiArray>();
            msg->data.assign(buf, buf + len);
            binary_pub_->publish(std::move(msg));
        }

        // Auto-detect protocol
        if (!use_ubx_ && !use_sbf_) {
            ubx_stream_.pushBytes(buf, len, ubx_events_);
            sbf_.decode(buf, len);
            detector_.detect(buf, len);
            if (detector_.isDetected()) {
                if (detector_.getProtocol() == ProtocolType::UBX) {
                    use_ubx_ = true;
                    RCLCPP_INFO(get_logger(), "Auto-detected UBX protocol");
                } else if (detector_.getProtocol() == ProtocolType::SBF) {
                    use_sbf_ = true;
                    RCLCPP_INFO(get_logger(), "Auto-detected SBF protocol");
                }
            }
            ubx_events_.clear();
            return;
        }

        if (use_ubx_) {
            ubx_events_.clear();
            ubx_stream_.pushBytes(buf, len, ubx_events_);
            for (const auto& ev : ubx_events_) {
                if (ev.has_nav_pvt) publishNavSatFix(ev.nav_pvt);
                if (ev.has_observation) publishRawEpoch(ev.observation);
            }
        } else if (use_sbf_) {
            auto msgs = sbf_.decode(buf, len);
            for (const auto& m : msgs) processSBF(m);
        }
    }

    void processSBF(const SBFMessage& msg) {
        if (msg.id == SBFBlockId::PVTGeodetic) {
            SBFPvtGeodetic pvt;
            if (sbf_.decodePvtGeodetic(msg, pvt) && !sbf_sentinel::isDoNotUse(pvt.latitude)) {
                auto fix = std::make_unique<sensor_msgs::msg::NavSatFix>();
                fix->header.stamp = now();
                fix->header.frame_id = frame_id_;
                fix->latitude = pvt.latitude * 180.0 / M_PI;
                fix->longitude = pvt.longitude * 180.0 / M_PI;
                fix->altitude = pvt.height;
                fix->status.status = (pvt.error == 0 && pvt.mode != 0) ?
                    sensor_msgs::msg::NavSatStatus::STATUS_FIX :
                    sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                fix->status.service = 15;
                fix_pub_->publish(std::move(fix));
            }
        }
    }

    void publishNavSatFix(const UBXNavPVT& pvt) {
        auto msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
        msg->header.stamp = now();
        msg->header.frame_id = frame_id_;
        msg->latitude = pvt.position_geodetic.latitude * 180.0 / M_PI;
        msg->longitude = pvt.position_geodetic.longitude * 180.0 / M_PI;
        msg->altitude = pvt.position_geodetic.height;

        if (!pvt.gnss_fix_ok || pvt.fix_type < 2) {
            msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        } else if (pvt.carrier_solution == 2) {
            msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        } else if (pvt.carrier_solution == 1) {
            msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        } else {
            msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        }
        msg->status.service = 15;
        msg->position_covariance_type =
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        double h_sig = pvt.horizontal_accuracy_m / 1.96;
        double v_sig = pvt.vertical_accuracy_m / 1.96;
        msg->position_covariance[0] = h_sig * h_sig;
        msg->position_covariance[4] = h_sig * h_sig;
        msg->position_covariance[8] = v_sig * v_sig;
        fix_pub_->publish(std::move(msg));
    }

    void publishRawEpoch(const ObservationData& obs) {
        auto msg = std::make_unique<gnss_raw_driver::msg::GnssRawEpoch>();
        msg->header.stamp = now();
        msg->header.frame_id = frame_id_;
        msg->gps_week = static_cast<uint16_t>(obs.time.week);
        msg->gps_tow = obs.time.tow;
        for (const auto& o : obs.observations) {
            gnss_raw_driver::msg::GnssRawObservation raw;
            raw.gnss_system = static_cast<uint8_t>(o.satellite.system);
            raw.prn = o.satellite.prn;
            raw.signal_type = static_cast<uint8_t>(o.signal);
            raw.pseudorange = o.pseudorange;
            raw.carrier_phase = o.carrier_phase;
            raw.doppler = o.doppler;
            raw.snr = static_cast<float>(o.snr);
            raw.valid = o.valid;
            msg->observations.push_back(raw);
        }
        msg->num_satellites = static_cast<uint8_t>(obs.observations.size());
        raw_pub_->publish(std::move(msg));
    }

    std::string device_;
    int baud_rate_;
    std::string protocol_str_;
    std::string frame_id_;
    bool publish_raw_binary_ = true;

    SerialPort serial_;
    UBXStreamDecoder ubx_stream_;
    std::vector<UBXStreamDecoder::Event> ubx_events_;
    SBFProcessor sbf_;
    ProtocolDetector detector_;
    bool use_ubx_ = false;
    bool use_sbf_ = false;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<gnss_raw_driver::msg::GnssRawEpoch>::SharedPtr raw_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr binary_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GnssRawDriverNode>());
    rclcpp::shutdown();
    return 0;
}
