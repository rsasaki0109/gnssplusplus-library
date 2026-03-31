/**
 * @file gnss_bag_processor_node.cpp
 * @brief ROS 2 node for offline processing of recorded GNSS raw binary data.
 *
 * Subscribes to /gnss/raw_binary (from ros2 bag play), decodes UBX/SBF,
 * and re-publishes parsed topics. Optionally writes .pos and .kml files.
 */
#include <fstream>
#include <iomanip>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "gnss_raw_driver/msg/gnss_raw_epoch.hpp"
#include "gnss_raw_driver/msg/gnss_raw_observation.hpp"

#include "libgnss++/io/ubx.hpp"
#include "libgnss++/io/sbf.hpp"
#include "libgnss++/io/protocol_detector.hpp"
#include "libgnss++/kml/kml.hpp"

using namespace libgnss;
using namespace libgnss::io;

class GnssBagProcessorNode : public rclcpp::Node {
public:
    GnssBagProcessorNode() : Node("gnss_bag_processor") {
        declare_parameter("protocol", "auto");
        declare_parameter("frame_id", "gnss");
        declare_parameter("output_pos", "");
        declare_parameter("output_kml", "");
        declare_parameter("kml_line_color_r", 1.0);
        declare_parameter("kml_line_color_g", 0.0);
        declare_parameter("kml_line_color_b", 0.0);

        protocol_str_ = get_parameter("protocol").as_string();
        frame_id_ = get_parameter("frame_id").as_string();
        output_pos_ = get_parameter("output_pos").as_string();
        output_kml_ = get_parameter("output_kml").as_string();
        kml_color_.r = get_parameter("kml_line_color_r").as_double();
        kml_color_.g = get_parameter("kml_line_color_g").as_double();
        kml_color_.b = get_parameter("kml_line_color_b").as_double();

        if (protocol_str_ == "ubx") use_ubx_ = true;
        else if (protocol_str_ == "sbf") use_sbf_ = true;

        fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("gnss/fix", 100);
        raw_pub_ = create_publisher<gnss_raw_driver::msg::GnssRawEpoch>("gnss/raw", 100);

        raw_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
            "gnss/raw_binary", 1000,
            std::bind(&GnssBagProcessorNode::rawCallback, this, std::placeholders::_1));

        if (!output_pos_.empty()) {
            pos_file_.open(output_pos_);
            if (pos_file_.is_open()) {
                pos_file_ << "# week,tow(s),lat(deg),lon(deg),height(m),fix_type,nSV,hAcc(m),vAcc(m)\n";
                pos_file_.flush();
                RCLCPP_INFO(get_logger(), "Writing .pos to %s", output_pos_.c_str());
            }
        }

        RCLCPP_INFO(get_logger(), "Bag processor ready. protocol=%s", protocol_str_.c_str());

        on_shutdown_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
            auto t = std::chrono::steady_clock::now();
            if (last_msg_time_.time_since_epoch().count() > 0 &&
                t - last_msg_time_ > std::chrono::seconds(2) &&
                !kml_written_ && !positions_.empty()) {
                writeOutputKml();
                kml_written_ = true;
                RCLCPP_INFO(get_logger(), "Processed %zu epochs, %zu positions",
                            epoch_count_, positions_.size());
            }
        });
    }

    ~GnssBagProcessorNode() override {
        if (!kml_written_ && !positions_.empty()) writeOutputKml();
        if (pos_file_.is_open()) pos_file_.close();
    }

private:
    void rawCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        last_msg_time_ = std::chrono::steady_clock::now();
        const uint8_t* data = msg->data.data();
        size_t len = msg->data.size();

        // Auto-detect
        if (!use_ubx_ && !use_sbf_) {
            ubx_stream_.pushBytes(data, len, ubx_events_);
            sbf_.decode(data, len);
            detector_.detect(data, len);
            if (detector_.isDetected()) {
                if (detector_.getProtocol() == ProtocolType::UBX) {
                    use_ubx_ = true;
                    RCLCPP_INFO(get_logger(), "Detected UBX protocol");
                } else if (detector_.getProtocol() == ProtocolType::SBF) {
                    use_sbf_ = true;
                    RCLCPP_INFO(get_logger(), "Detected SBF protocol");
                }
            }
            ubx_events_.clear();
            return;
        }

        if (use_ubx_) {
            ubx_events_.clear();
            ubx_stream_.pushBytes(data, len, ubx_events_);
            for (const auto& ev : ubx_events_) {
                if (ev.has_nav_pvt) recordAndPublishPvt(ev.nav_pvt);
                if (ev.has_observation) {
                    publishRawEpoch(ev.observation);
                    ++epoch_count_;
                }
            }
        } else if (use_sbf_) {
            auto msgs = sbf_.decode(data, len);
            for (const auto& m : msgs) {
                if (m.id == SBFBlockId::PVTGeodetic) {
                    SBFPvtGeodetic pvt;
                    if (sbf_.decodePvtGeodetic(m, pvt) && !sbf_sentinel::isDoNotUse(pvt.latitude)) {
                        double lat_deg = pvt.latitude * 180.0 / M_PI;
                        double lon_deg = pvt.longitude * 180.0 / M_PI;
                        positions_.push_back({lat_deg, lon_deg, pvt.height});
                        // Publish NavSatFix
                        auto fix = std::make_unique<sensor_msgs::msg::NavSatFix>();
                        fix->header.stamp = now();
                        fix->header.frame_id = frame_id_;
                        fix->latitude = lat_deg;
                        fix->longitude = lon_deg;
                        fix->altitude = pvt.height;
                        fix->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                        fix->status.service = 15;
                        fix_pub_->publish(std::move(fix));
                    }
                }
            }
        }
    }

    void recordAndPublishPvt(const UBXNavPVT& pvt) {
        double lat_deg = pvt.position_geodetic.latitude * 180.0 / M_PI;
        double lon_deg = pvt.position_geodetic.longitude * 180.0 / M_PI;

        // Publish NavSatFix
        auto fix = std::make_unique<sensor_msgs::msg::NavSatFix>();
        fix->header.stamp = now();
        fix->header.frame_id = frame_id_;
        fix->latitude = lat_deg;
        fix->longitude = lon_deg;
        fix->altitude = pvt.position_geodetic.height;

        if (!pvt.gnss_fix_ok || pvt.fix_type < 2) {
            fix->status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        } else if (pvt.carrier_solution == 2) {
            fix->status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        } else {
            fix->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        }
        fix->status.service = 15;
        double h_sig = pvt.horizontal_accuracy_m / 1.96;
        double v_sig = pvt.vertical_accuracy_m / 1.96;
        fix->position_covariance_type =
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        fix->position_covariance[0] = h_sig * h_sig;
        fix->position_covariance[4] = h_sig * h_sig;
        fix->position_covariance[8] = v_sig * v_sig;
        fix_pub_->publish(std::move(fix));

        // Write .pos
        if (pos_file_.is_open() && pvt.gnss_fix_ok && pvt.fix_type >= 2) {
            pos_file_ << std::fixed
                      << pvt.time.week << ","
                      << std::setprecision(3) << pvt.time.tow << ","
                      << std::setprecision(9) << lat_deg << ","
                      << std::setprecision(9) << lon_deg << ","
                      << std::setprecision(4) << pvt.position_geodetic.height << ","
                      << static_cast<int>(pvt.fix_type) << ","
                      << static_cast<int>(pvt.num_sv) << ","
                      << std::setprecision(3) << pvt.horizontal_accuracy_m << ","
                      << pvt.vertical_accuracy_m << "\n";
            pos_file_.flush();
        }

        // Store for KML
        if (pvt.gnss_fix_ok && pvt.fix_type >= 2) {
            positions_.push_back({lat_deg, lon_deg, pvt.position_geodetic.height});
        }
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

    void writeOutputKml() {
        if (output_kml_.empty() || positions_.empty()) return;
        using namespace libgnss::kml;
        std::vector<Location> locs;
        for (const auto& p : positions_) locs.push_back({p[0], p[1], p[2]});
        std::string content = createLine("Trajectory", locs, 3.0, kml_color_, 0.9, 0,
                                         AltitudeMode::ClampToGround);
        if (locs.size() >= 2) {
            content += createPoints({"Start", "End"}, {locs.front(), locs.back()},
                                    1.5, {Color{0,1,0}, Color{1,0,0}}, 1.0,
                                    AltitudeMode::ClampToGround);
        }
        if (libgnss::kml::writeKml(output_kml_, content, "GNSS Trajectory")) {
            RCLCPP_INFO(get_logger(), "KML written to %s (%zu points)",
                        output_kml_.c_str(), positions_.size());
        }
    }

    std::string protocol_str_, frame_id_, output_pos_, output_kml_;
    libgnss::kml::Color kml_color_{1.0, 0.0, 0.0};

    UBXStreamDecoder ubx_stream_;
    std::vector<UBXStreamDecoder::Event> ubx_events_;
    SBFProcessor sbf_;
    ProtocolDetector detector_;
    bool use_ubx_ = false, use_sbf_ = false;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<gnss_raw_driver::msg::GnssRawEpoch>::SharedPtr raw_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_sub_;

    std::ofstream pos_file_;
    std::vector<std::array<double, 3>> positions_;
    size_t epoch_count_ = 0;
    std::chrono::steady_clock::time_point last_msg_time_;
    rclcpp::TimerBase::SharedPtr on_shutdown_;
    bool kml_written_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GnssBagProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
