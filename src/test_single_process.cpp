// test_single_process.cpp
//
// Loads the full config_template.yaml (all topic types) and, for each
// configured LowpassFilterNode, spins up a matching test publisher and
// subscriber in the SAME process, so no cross-process DDS discovery is
// required. Publishes synthetic data to every topic and prints what each
// filter node produces on its output topic.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "lowpass_filter_node.h"
#include "config_loader.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

using sensor_msgs::msg::JointState;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Float64;
using std_msgs::msg::Float64MultiArray;
using sensor_msgs::msg::FluidPressure;
using sensor_msgs::msg::MagneticField;
using namespace std::chrono_literals;

// --- generic test helpers, reused across every message type ---

template <typename MsgT>
class GenericPublisher : public rclcpp::Node {
public:
    GenericPublisher(const std::string& node_name, const std::string& topic)
        : Node(node_name) {
        pub_ = create_publisher<MsgT>(topic, 10);
    }

    void publish_msg(const MsgT& msg) {
        pub_->publish(msg);
    }

private:
    typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
};

template <typename MsgT>
class GenericSubscriber : public rclcpp::Node {
public:
    GenericSubscriber(const std::string& node_name, const std::string& topic)
        : Node(node_name) {
        sub_ = create_subscription<MsgT>(
            topic, 10,
            [this](const typename MsgT::SharedPtr msg) {
                last_msg_ = *msg;
                received_ = true;
            });
    }

    bool received_ = false;
    MsgT last_msg_{};

private:
    typename rclcpp::Subscription<MsgT>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config_template.yaml>\n";
        return 1;
    }

    auto configs = load_configs(argv[1]);

    rclcpp::executors::SingleThreadedExecutor executor;

    std::vector<std::shared_ptr<LowpassFilterNode>> filter_nodes;

    // one set of vectors per message type; index i across each type's
    // vectors corresponds to the same configured node
    std::vector<std::shared_ptr<GenericPublisher<JointState>>> js_pubs;
    std::vector<std::shared_ptr<GenericSubscriber<JointState>>> js_subs;
    std::vector<JointStateMode> js_modes;

    std::vector<std::shared_ptr<GenericPublisher<Odometry>>> odom_pubs;
    std::vector<std::shared_ptr<GenericSubscriber<Odometry>>> odom_subs;

    std::vector<std::shared_ptr<GenericPublisher<Float64>>> f64_pubs;
    std::vector<std::shared_ptr<GenericSubscriber<Float64>>> f64_subs;

    std::vector<std::shared_ptr<GenericPublisher<Float64MultiArray>>> f64ma_pubs;
    std::vector<std::shared_ptr<GenericSubscriber<Float64MultiArray>>> f64ma_subs;
    std::vector<size_t> f64ma_sizes;

    std::vector<std::shared_ptr<GenericPublisher<FluidPressure>>> fp_pubs;
    std::vector<std::shared_ptr<GenericSubscriber<FluidPressure>>> fp_subs;

    std::vector<std::shared_ptr<GenericPublisher<MagneticField>>> mf_pubs;
    std::vector<std::shared_ptr<GenericSubscriber<MagneticField>>> mf_subs;

    int idx = 0;
    for (const auto& conf : configs) {
        auto filter_node = std::make_shared<LowpassFilterNode>(conf.node_name, conf.topic_name, conf.config);
        filter_nodes.push_back(filter_node);
        executor.add_node(filter_node);

        std::string tag = "_" + std::to_string(idx++);

        switch (conf.config.topic_type) {
            case TopicType::JointState: {
                auto pub = std::make_shared<GenericPublisher<JointState>>("js_pub" + tag, conf.topic_name);
                auto sub = std::make_shared<GenericSubscriber<JointState>>("js_sub" + tag, conf.topic_name + "_filt");
                executor.add_node(pub);
                executor.add_node(sub);
                js_pubs.push_back(pub);
                js_subs.push_back(sub);
                js_modes.push_back(conf.config.mode);
                break;
            }
            case TopicType::Odometry: {
                auto pub = std::make_shared<GenericPublisher<Odometry>>("odom_pub" + tag, conf.topic_name);
                auto sub = std::make_shared<GenericSubscriber<Odometry>>("odom_sub" + tag, conf.topic_name + "_filt");
                executor.add_node(pub);
                executor.add_node(sub);
                odom_pubs.push_back(pub);
                odom_subs.push_back(sub);
                break;
            }
            case TopicType::Float64: {
                auto pub = std::make_shared<GenericPublisher<Float64>>("f64_pub" + tag, conf.topic_name);
                auto sub = std::make_shared<GenericSubscriber<Float64>>("f64_sub" + tag, conf.topic_name + "_filt");
                executor.add_node(pub);
                executor.add_node(sub);
                f64_pubs.push_back(pub);
                f64_subs.push_back(sub);
                break;
            }
            case TopicType::Float64MultiArray: {
                auto pub = std::make_shared<GenericPublisher<Float64MultiArray>>("f64ma_pub" + tag, conf.topic_name);
                auto sub = std::make_shared<GenericSubscriber<Float64MultiArray>>("f64ma_sub" + tag, conf.topic_name + "_filt");
                executor.add_node(pub);
                executor.add_node(sub);
                f64ma_pubs.push_back(pub);
                f64ma_subs.push_back(sub);
                f64ma_sizes.push_back(conf.config.filters.size());
                break;
            }
            case TopicType::FluidPressure: {
                auto pub = std::make_shared<GenericPublisher<FluidPressure>>("fp_pub" + tag, conf.topic_name);
                auto sub = std::make_shared<GenericSubscriber<FluidPressure>>("fp_sub" + tag, conf.topic_name + "_filt");
                executor.add_node(pub);
                executor.add_node(sub);
                fp_pubs.push_back(pub);
                fp_subs.push_back(sub);
                break;
            }
            case TopicType::MagneticField: {
                auto pub = std::make_shared<GenericPublisher<MagneticField>>("mf_pub" + tag, conf.topic_name);
                auto sub = std::make_shared<GenericSubscriber<MagneticField>>("mf_sub" + tag, conf.topic_name + "_filt");
                executor.add_node(pub);
                executor.add_node(sub);
                mf_pubs.push_back(pub);
                mf_subs.push_back(sub);
                break;
            }
        }
    }

    const int n_steps = 20;
    for (int i = 0; i < n_steps; i++) {
        double t = 0.3 * i;

        // JointState: raw_encoder needs a value in the encoder's raw range
        // (~100-2781, matching conversions.h); full_state just needs small
        // plausible position/velocity/effort values.
        for (size_t k = 0; k < js_pubs.size(); k++) {
            JointState msg;
            msg.name = {"test_joint"};
            if (js_modes[k] == JointStateMode::RawEncoder) {
                double raw = 100.0 + 1340.0 * (1.0 + std::sin(t));
                msg.position = {raw};
            } else {
                msg.position = {std::sin(t)};
                msg.velocity = {std::cos(t)};
                msg.effort = {std::sin(t) * 0.5};
            }
            js_pubs[k]->publish_msg(msg);
        }

        for (size_t k = 0; k < odom_pubs.size(); k++) {
            Odometry msg;
            msg.twist.twist.linear.x = std::sin(t);
            msg.twist.twist.angular.z = std::cos(t);
            odom_pubs[k]->publish_msg(msg);
        }

        for (size_t k = 0; k < f64_pubs.size(); k++) {
            Float64 msg;
            msg.data = std::sin(t);
            f64_pubs[k]->publish_msg(msg);
        }

        for (size_t k = 0; k < f64ma_pubs.size(); k++) {
            Float64MultiArray msg;
            msg.data.resize(f64ma_sizes[k]);
            for (size_t j = 0; j < f64ma_sizes[k]; j++) {
                msg.data[j] = std::sin(t + j);
            }
            f64ma_pubs[k]->publish_msg(msg);
        }

        // FluidPressure: sine wave around a nominal atmospheric pressure
        for (size_t k = 0; k < fp_pubs.size(); k++) {
            FluidPressure msg;
            msg.fluid_pressure = 101325.0 + 100.0 * std::sin(t);
            msg.variance = 0.0;
            fp_pubs[k]->publish_msg(msg);
        }

        // MagneticField: small sine-based values on each axis
        for (size_t k = 0; k < mf_pubs.size(); k++) {
            MagneticField msg;
            msg.magnetic_field.x = std::sin(t) * 1e-5;
            msg.magnetic_field.y = std::cos(t) * 1e-5;
            msg.magnetic_field.z = std::sin(t * 0.5) * 1e-5;
            mf_pubs[k]->publish_msg(msg);
        }

        executor.spin_some(50ms);
        std::this_thread::sleep_for(20ms);
        executor.spin_some(50ms);
    }

    std::cout << "\n=== final filtered values ===\n";
    for (size_t k = 0; k < js_subs.size(); k++) {
        const auto& m = js_subs[k]->last_msg_;
        std::cout << "JointState[" << k << "] position=" << (m.position.empty() ? 0.0 : m.position[0])
                   << " velocity=" << (m.velocity.empty() ? 0.0 : m.velocity[0])
                   << " effort=" << (m.effort.empty() ? 0.0 : m.effort[0]) << "\n";
    }
    for (size_t k = 0; k < odom_subs.size(); k++) {
        const auto& m = odom_subs[k]->last_msg_;
        std::cout << "Odometry[" << k << "] vx=" << m.twist.twist.linear.x
                   << " wz=" << m.twist.twist.angular.z << "\n";
    }
    for (size_t k = 0; k < f64_subs.size(); k++) {
        std::cout << "Float64[" << k << "] value=" << f64_subs[k]->last_msg_.data << "\n";
    }
    for (size_t k = 0; k < f64ma_subs.size(); k++) {
        std::cout << "Float64MultiArray[" << k << "] data=[";
        for (double v : f64ma_subs[k]->last_msg_.data) {
            std::cout << v << " ";
        }
        std::cout << "]\n";
    }
    for (size_t k = 0; k < fp_subs.size(); k++) {
        std::cout << "FluidPressure[" << k << "] fluid_pressure=" << fp_subs[k]->last_msg_.fluid_pressure << "\n";
    }
    for (size_t k = 0; k < mf_subs.size(); k++) {
        const auto& m = mf_subs[k]->last_msg_;
        std::cout << "MagneticField[" << k << "] x=" << m.magnetic_field.x
                   << " y=" << m.magnetic_field.y
                   << " z=" << m.magnetic_field.z << "\n";
    }

    rclcpp::shutdown();
    return 0;
}