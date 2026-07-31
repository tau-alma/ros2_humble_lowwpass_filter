// lowpass_filter_node.h
#ifndef LOWPASS_FILTER_NODE_H
#define LOWPASS_FILTER_NODE_H

// ros2 includes
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

// standard imports
#include <chrono>
#include <string>
#include <map>
#include <cmath>
#include <stdexcept>

#include "lowpass_filter.h"
#include "conversions.h"

enum class TopicType {
    JointState,
    Odometry,
    Float64,
    Float64MultiArray,
    MagneticField,
    FluidPressure
};

enum class SensorType {
    CenterLink,
    Boom,
    Bucket,
    None
};

enum class JointStateMode {
    RawEncoder,
    FullState
};

struct FilterConfig {
    double freq;
    double cutoff;
    double zeta = 1/std::sqrt(2);
    int order = 1;
    bool derivator = false;
    bool prewarp = false;
};

struct FilterNodeConfig {
    TopicType topic_type;
    SensorType sensor;                 
    JointStateMode mode;                
    std::map<std::string, FilterConfig> filters; 
};



class LowpassFilterNode : public rclcpp::Node {
    public:
        LowpassFilterNode(const std::string& node_name, const std::string& topic_name, const FilterNodeConfig& config);

        
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void float64_callback(const std_msgs::msg::Float64::SharedPtr msg);
        void float64_multiarray_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        void magnetic_field_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
        void fluid_pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);
    private:
        JointStateMode joint_state_mode_;
        SensorType sensor_;
        std::map<std::string, LowpassFilter> filters_;

        // Initialize subscribers

        sensor_msgs::msg::JointState joint_state_out_msg_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

        nav_msgs::msg::Odometry odometry_msg_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

        std_msgs::msg::Float64 float64_msg_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr float64_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr float64_pub_;

        std_msgs::msg::Float64MultiArray float64_multiarray_msg_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr float64_multiarray_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr float64_multiarray_pub_;

        sensor_msgs::msg::MagneticField magnetic_field_msg_;
        rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_field_sub_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_field_pub_;

        sensor_msgs::msg::FluidPressure fluid_pressure_msg_;
        rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr fluid_pressure_sub_;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr fluid_pressure_pub_;

};


#endif