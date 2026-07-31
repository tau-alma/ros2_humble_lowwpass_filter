// config_loader.h
#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "lowpass_filter_node.h"

TopicType parse_topic_type(const std::string& s);
SensorType parse_sensor_type(const std::string& s);
JointStateMode parse_joint_state_mode(const std::string& s);
FilterConfig parse_filter_config(const YAML::Node& node);

struct NodeConf {
    std::string node_name;
    std::string topic_name;
    FilterNodeConfig config;
};

std::vector<NodeConf> load_configs(const std::string& yaml_path);

#endif