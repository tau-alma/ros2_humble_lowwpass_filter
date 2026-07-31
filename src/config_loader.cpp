// config_loader.cpp

#include "config_loader.h"

#include <stdexcept>

TopicType parse_topic_type(const std::string& s) {
    if (s == "joint_state") {
        return TopicType::JointState;
    } else if (s == "odometry") {
        return TopicType::Odometry;
    } else if (s == "float64") {
        return TopicType::Float64;
    } else if (s == "float64_multi_array") {
        return TopicType::Float64MultiArray;
    } else if (s == "magnetic_field") {
        return TopicType::MagneticField;
    } else if (s == "fluid_pressure") {
        return TopicType::FluidPressure;
    } else {
        throw std::invalid_argument("Unknown topic type: " + s);
    }
}

SensorType parse_sensor_type(const std::string& s) {
    if (s == "center_link") {
        return SensorType::CenterLink;
    } else if (s == "boom") {
        return SensorType::Boom;
    } else if (s == "bucket") {
        return SensorType::Bucket;
    } else if (s == "none") {
        return SensorType::None;
    } else {
        throw std::invalid_argument("Unknown sensor type: " + s);
    }
}

JointStateMode parse_joint_state_mode(const std::string& s) {
    if (s == "raw_encoder") {
        return JointStateMode::RawEncoder;
    } else if (s == "full_state") {
        return JointStateMode::FullState;
    } else {
        throw std::invalid_argument("Unknown joint state mode: " + s);
    }
}

FilterConfig parse_filter_config(const YAML::Node& node) {
    FilterConfig config;
    config.freq = node["freq"].as<double>();
    config.cutoff = node["cutoff"].as<double>();

    if (node["zeta"]) {
        config.zeta = node["zeta"].as<double>();
    }
    if (node["order"]) {
        config.order = node["order"].as<int>();
    }
    if (node["derivator"]) {
        config.derivator = node["derivator"].as<bool>();
    }
    if (node["prewarp"]) {
        config.prewarp = node["prewarp"].as<bool>();
    }
    return config;
}

std::vector<NodeConf> load_configs(const std::string& yaml_path) {
    std::vector<NodeConf> configs;

    YAML::Node root = YAML::LoadFile(yaml_path);

    for (const auto& entry : root) {
        YAML::Node node_config = entry.second;

        NodeConf conf;
        conf.node_name = node_config["node_name"].as<std::string>();
        conf.topic_name = node_config["topic_name"].as<std::string>();
        conf.config.topic_type = parse_topic_type(node_config["topic_type"].as<std::string>());

        if (conf.config.topic_type == TopicType::JointState) {
            conf.config.sensor = parse_sensor_type(node_config["sensor"].as<std::string>());
            conf.config.mode = parse_joint_state_mode(node_config["mode"].as<std::string>());
        }

        for (const auto& filter_entry : node_config["filters"]) {
            std::string filter_key = filter_entry.first.as<std::string>();
            conf.config.filters[filter_key] = parse_filter_config(filter_entry.second);
        }

        configs.push_back(conf);
    }

    return configs;
}