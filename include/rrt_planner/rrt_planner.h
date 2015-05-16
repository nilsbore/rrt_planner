/** include the libraries you need in your planner here */
/** for global path planner interface */
#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

using std::string;

namespace rrt_planner {

struct tree_node {
    geometry_msgs::Pose pose;
    double accum_cost;
    int parent_idx;
};

class RRTPlanner : public nav_core::BaseGlobalPlanner {
private:
    double increment_dist;
    double min_turn_radius;
    ros::NodeHandle pn;
    ros::Publisher pub;

    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    base_local_planner::WorldModel* world_model_;
    bool initialized_;
    std::vector<geometry_msgs::Point> footprint;

    double cost(const geometry_msgs::Pose& pose);
    int nearest(const geometry_msgs::Pose& pose, const std::vector<tree_node>& nodes);
    geometry_msgs::Pose sample();
    geometry_msgs::Pose generate_control(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);
    void publish_display_tree_message(const std::vector<tree_node>& nodes);
public:

    RRTPlanner();
    RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
};

} // namespace rrt_planner
 #endif
