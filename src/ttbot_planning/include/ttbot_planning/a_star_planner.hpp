#ifndef A_STAR_PLANNER_HPP
#define A_STAR_PLANNER_HPP

#include <memory>
#include <vector>
#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ttbot_planning
{
struct GraphNode {
    int x, y, cost;
    double heuristic;
    std::shared_ptr<GraphNode> prev;
    GraphNode(int in_x = 0, int in_y = 0) : x(in_x), y(in_y), cost(0), heuristic(0.0), prev(nullptr) {}
    bool operator>(const GraphNode & other) const { return cost + heuristic > other.cost + other.heuristic; }
    bool operator==(const GraphNode & other) const { return x == other.x && y == other.y; }
    GraphNode operator+(const std::pair<int, int> & dir) const { return GraphNode(x + dir.first, y + dir.second); }
};

class AStarPlanner : public rclcpp::Node {
public:
    AStarPlanner();
private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal);
    bool poseOnMap(const GraphNode & node);
    GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);
    geometry_msgs::msg::Pose gridToWorld(const GraphNode & node);
    unsigned int poseToCell(const GraphNode & node);
    double manhattanDistance(const GraphNode &node, const GraphNode &goal_node);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_map_pub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::OccupancyGrid visited_map_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};
}
#endif