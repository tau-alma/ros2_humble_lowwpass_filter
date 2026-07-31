// main.cpp

#include "rclcpp/rclcpp.hpp"
#include "lowpass_filter_node.h"
#include "config_loader.h"

#include <iostream>
#include <vector>
#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config.yaml>\n";
        return 1;
    }

    auto configs = load_configs(argv[1]);

    rclcpp::executors::MultiThreadedExecutor executor;
    std::vector<std::shared_ptr<LowpassFilterNode>> nodes;

    for (const auto& conf : configs) {
        auto node = std::make_shared<LowpassFilterNode>(conf.node_name, conf.topic_name, conf.config);
        nodes.push_back(node);
        executor.add_node(node);
    }

    std::cout << "spinning " << nodes.size() << " filter node(s)\n";
    executor.spin();

    rclcpp::shutdown();
    return 0;
}