// lowpass_filter_node.cpp

#include "lowpass_filter_node.h"

LowpassFilterNode::LowpassFilterNode(
    const std::string& node_name, 
    const std::string& topic_name,
    const FilterNodeConfig& config)
    : Node(node_name),
    joint_state_mode_(config.mode),
    sensor_(config.sensor) {

        if (config.topic_type == TopicType::JointState) {
            joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
                topic_name, 10, std::bind(&LowpassFilterNode::joint_state_callback, this, std::placeholders::_1)
            );
            joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(topic_name + "_filt", 10);
            
            if (config.mode == JointStateMode::RawEncoder) {
                for (const auto& key : {"ang", "vel", "acc"}) {
                    const FilterConfig& filter_config = config.filters.at(key);
                    filters_.emplace(key, LowpassFilter(filter_config.freq, filter_config.cutoff, filter_config.zeta,
                                                        filter_config.order, filter_config.derivator, filter_config.prewarp));
                }
            } else if (config.mode == JointStateMode::FullState) {
                for (const auto& key : {"position", "velocity", "effort"}) {
                    const FilterConfig& filter_config = config.filters.at(key);
                    filters_.emplace(key, LowpassFilter(filter_config.freq, filter_config.cutoff, filter_config.zeta,
                                                        filter_config.order, filter_config.derivator, filter_config.prewarp));
                } 
            } else {
                RCLCPP_ERROR(get_logger(), "LowpassFilterNode: unsupported joint state mode");
                throw std::invalid_argument("LowpassFilterNode: unsupported joint state mode");
            }
            
        } else if (config.topic_type == TopicType::Odometry) {
            odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                topic_name, 10, std::bind(&LowpassFilterNode::odometry_callback, this, std::placeholders::_1)
            );
            odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(topic_name + "_filt", 10);      
            
            for (const auto& [key, filter_config] : config.filters) {
                filters_.emplace(key, LowpassFilter(filter_config.freq, filter_config.cutoff, filter_config.zeta,
                                                        filter_config.order, filter_config.derivator, filter_config.prewarp));
            }

        } else if (config.topic_type == TopicType::Float64) {
            float64_sub_ = create_subscription<std_msgs::msg::Float64>(
                topic_name, 10, std::bind(&LowpassFilterNode::float64_callback, this, std::placeholders::_1)
            );
            float64_pub_ = create_publisher<std_msgs::msg::Float64>(topic_name + "_filt", 10); 
            const FilterConfig& filter_config = config.filters.at("value");
            filters_.emplace("value", LowpassFilter(filter_config.freq, filter_config.cutoff, filter_config.zeta,
                                                        filter_config.order, filter_config.derivator, filter_config.prewarp));

        } else if (config.topic_type == TopicType::Float64MultiArray) {
            float64_multiarray_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
                topic_name, 10, std::bind(&LowpassFilterNode::float64_multiarray_callback, this, std::placeholders::_1)
            );
            float64_multiarray_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(topic_name + "_filt", 10); 
            
            for (const auto& [key, filter_config] : config.filters) {
                filters_.emplace(key, LowpassFilter(filter_config.freq, filter_config.cutoff, filter_config.zeta,
                                                        filter_config.order, filter_config.derivator, filter_config.prewarp));
            }
            float64_multiarray_msg_.data.resize(filters_.size());

        } else if (config.topic_type == TopicType::MagneticField) {
            magnetic_field_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
                topic_name, 10, std::bind(&LowpassFilterNode::magnetic_field_callback, this, std::placeholders::_1)
            );
            magnetic_field_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(topic_name + "_filt", 10);

            for (const auto& [key, filter_config] : config.filters) {
                filters_.emplace(key, LowpassFilter(filter_config.freq, filter_config.cutoff, filter_config.zeta,
                                                    filter_config.order, filter_config.derivator, filter_config.prewarp));
            }

        } else if (config.topic_type == TopicType::FluidPressure) {
            fluid_pressure_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
                topic_name, 10, std::bind(&LowpassFilterNode::fluid_pressure_callback, this, std::placeholders::_1)
            );
            fluid_pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(topic_name + "_filt", 10);


            const FilterConfig& filter_config = config.filters.at("fluid_pressure");
                filters_.emplace("fluid_pressure", LowpassFilter(filter_config.freq, filter_config.cutoff, filter_config.zeta,
                                                    filter_config.order, filter_config.derivator, filter_config.prewarp));

        } else {
            RCLCPP_ERROR(get_logger(), "LowpassFilterNode: unsupported topic type");
            throw std::invalid_argument("LowpassFilterNode: unsupported topic type");
        }
        
        
        
};



void LowpassFilterNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (joint_state_mode_ == JointStateMode::RawEncoder) {
        double angle_rad = msg->position[0];
        double angle_deg = 0.0;  

        switch (sensor_) {
            case SensorType::CenterLink: {
                angle_deg = convert_center_link(angle_rad);
                break;
            }
            case SensorType::Boom: {
                angle_deg = convert_boom(angle_rad);
                break;
            }
            case SensorType::Bucket: {
                angle_deg = convert_bucket(angle_rad);
                break;
            }
            default: {
                RCLCPP_ERROR(get_logger(), "LowpassFilterNode: unsupported SensorType for joint state message");
                throw std::invalid_argument("LowpassFilterNode: unsupported SensorType for joint state message");
            }
        }
        
        double angle_filt = filters_.at("ang").step(angle_deg);
        double velocity_filt = filters_.at("vel").step(angle_filt);
        double acceleration_filt = filters_.at("acc").step(velocity_filt);

        joint_state_out_msg_.header.stamp = msg->header.stamp;
        joint_state_out_msg_.position = {angle_filt};
        joint_state_out_msg_.velocity = {velocity_filt};
        joint_state_out_msg_.effort = {acceleration_filt};

        joint_state_pub_->publish(joint_state_out_msg_);

    } else if (joint_state_mode_ == JointStateMode::FullState) {
        double angle_raw = msg->position[0];
        double velocity_raw = msg->velocity[0];
        double effort_raw = msg->effort[0];

        double angle_filt = filters_.at("position").step(angle_raw);
        double velocity_filt = filters_.at("velocity").step(velocity_raw);
        double acceleration_filt = filters_.at("effort").step(effort_raw);

        joint_state_out_msg_.header.stamp = msg->header.stamp;
        joint_state_out_msg_.position = {angle_filt};
        joint_state_out_msg_.velocity = {velocity_filt};
        joint_state_out_msg_.effort = {acceleration_filt};

        joint_state_pub_->publish(joint_state_out_msg_);
    }
}


void LowpassFilterNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    
    for (auto& [key, filter] : filters_) {
        if (key == "vx") {
            msg->twist.twist.linear.x = filter.step(msg->twist.twist.linear.x);
        } else if (key == "vy") {
            msg->twist.twist.linear.y = filter.step(msg->twist.twist.linear.y);
        } else if (key == "vz") {
            msg->twist.twist.linear.z = filter.step(msg->twist.twist.linear.z);
        } else if (key == "wx") {
            msg->twist.twist.angular.x = filter.step(msg->twist.twist.angular.x);
        } else if (key == "wy") {
            msg->twist.twist.angular.y = filter.step(msg->twist.twist.angular.y);
        } else if (key == "wz") {
            msg->twist.twist.angular.z = filter.step(msg->twist.twist.angular.z);
        } else {
            RCLCPP_ERROR(get_logger(), "LowpassFilterNode: unsupported key '%s' for odometry message", key.c_str());
            throw std::invalid_argument("LowpassFilterNode: unsupported key '" + key + "' for odometry message");
        }
    }
    odometry_msg_ = *msg;
    odometry_pub_->publish(odometry_msg_);
}


void LowpassFilterNode::float64_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    
    float64_msg_.data = filters_.at("value").step(msg->data);
    float64_pub_->publish(float64_msg_);
}


void LowpassFilterNode::float64_multiarray_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    
    if (msg->data.size() != filters_.size()) {
        RCLCPP_ERROR(get_logger(), "LowpassFilterNode: Float64MultiArray size mismatch");
            throw std::invalid_argument("LowpassFilterNode: Float64MultiArray size mismatch");
    }
    for (size_t i = 0; i < msg->data.size(); i++) {
        std::string key = std::to_string(i);
        float64_multiarray_msg_.data[i] = filters_.at(key).step(msg->data[i]);
    }

    float64_multiarray_pub_->publish(float64_multiarray_msg_);
}

void LowpassFilterNode::magnetic_field_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    for (auto& [key, filter] : filters_) {
        if (key == "x") {
            msg->magnetic_field.x = filter.step(msg->magnetic_field.x);
        } else if (key == "y") {
            msg->magnetic_field.y = filter.step(msg->magnetic_field.y);
        } else if (key == "z") {
            msg->magnetic_field.z = filter.step(msg->magnetic_field.z);
        } else {
            RCLCPP_ERROR(get_logger(), "LowpassFilterNode: unsupported key '%s' for magneticfield message", key.c_str());
            throw std::invalid_argument("LowpassFilterNode: unsupported key '" + key + "' for magneticfield message");
        }
    }
    
    magnetic_field_msg_ = *msg;
    magnetic_field_pub_->publish(magnetic_field_msg_);
}

void LowpassFilterNode::fluid_pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
    msg->fluid_pressure = filters_.at("fluid_pressure").step(msg->fluid_pressure);
    fluid_pressure_msg_ = *msg;
    fluid_pressure_pub_->publish(fluid_pressure_msg_);
}