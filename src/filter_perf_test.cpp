// main.cpp
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "lowpass_filter.h"  // adjust to your actual include path

using sensor_msgs::msg::JointState;

constexpr double FS = 100.0;
constexpr double CF = 5.0;
constexpr int N_CALLS = 100000;

class FilterPerfTestNode : public rclcpp::Node {
public:
    FilterPerfTestNode()
        : Node("filter_perf_test"),
          ang_filter(FS, CF),
          vel_filter(FS, CF),
          acc_filter(FS, CF) {
        publisher_ = create_publisher<JointState>("joint_states_filt", 10);
        msg_.name = {"test_joint"};
        msg_.position.resize(1);
        msg_.velocity.resize(1);
        msg_.effort.resize(1);
    }

    LowpassFilter ang_filter;
    LowpassFilter vel_filter;
    LowpassFilter acc_filter;
    JointState msg_;
    rclcpp::Publisher<JointState>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FilterPerfTestNode>();

    long long filter_total_ns = 0;
    long long publish_total_ns = 0;

    for (int i = 0; i < N_CALLS; i++) {
        double angle_rad = 0.01 * i;  // synthetic ramp input

        auto t0 = std::chrono::steady_clock::now();
        double angle_filt = node->ang_filter.step(angle_rad);
        double vel_filt = node->vel_filter.step(angle_rad);
        double acc_filt = node->acc_filter.step(vel_filt);
        auto t1 = std::chrono::steady_clock::now();

        node->msg_.position[0] = angle_filt;
        node->msg_.velocity[0] = vel_filt;
        node->msg_.effort[0] = acc_filt;

        auto t2 = std::chrono::steady_clock::now();
        node->publisher_->publish(node->msg_);
        auto t3 = std::chrono::steady_clock::now();

        filter_total_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
        publish_total_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2).count();
    }

    std::cout << "=== C++ (rclcpp), " << N_CALLS << " iterations ===\n";
    std::cout << "  avg filter compute: "
              << static_cast<double>(filter_total_ns) / N_CALLS << " ns\n";
    std::cout << "  avg publish() call: "
              << static_cast<double>(publish_total_ns) / N_CALLS << " ns\n";

    rclcpp::shutdown();
    return 0;
}