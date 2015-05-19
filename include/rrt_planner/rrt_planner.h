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
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <people_msgs/People.h>

using std::string;

namespace rrt_planner {

struct tree_node {
    geometry_msgs::Pose pose;
    double accum_cost;
    int parent_idx;
    double time_elapsed;
};

class RRTPlanner : public nav_core::BaseGlobalPlanner {
public:

    using PointT = pcl::PointXYZ;
    using CloudT = pcl::PointCloud<PointT>;

private:
    double increment_dist;
    double min_turn_radius;
    double travel_cost;
    double mean_speed;
    std::vector<people_msgs::Person> people;
    std::vector<people_msgs::Person> last_people;
    ros::NodeHandle pn;
    ros::Publisher tree_pub;
    ros::Subscriber people_sub;

    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    base_local_planner::WorldModel* world_model_;
    bool initialized_;
    std::vector<geometry_msgs::Point> footprint;

    double cost(const geometry_msgs::Pose& pose);
    int nearest(const geometry_msgs::Pose& pose, pcl::octree::OctreePointCloudSearch<PointT>& octree);
    geometry_msgs::Pose sample();
    std::pair<geometry_msgs::Pose, double> steer_towards(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);
    std::pair<geometry_msgs::Pose, double> steer(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);
    void publish_display_tree_message(const std::vector<tree_node>& nodes);
    std::vector<int> near(const geometry_msgs::Pose& pose, const pcl::octree::OctreePointCloudSearch<PointT>& octree);
    tree_node choose_parent(const geometry_msgs::Pose& towards_sampled, std::vector<int>& T,
                            int initial_parent, std::vector<tree_node>& nodes);
    void rewire(const tree_node& new_node, int new_ind, std::vector<int>& T,
                std::vector<tree_node>& nodes);
    std::vector<geometry_msgs::Point> generate_local_path(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);
    void publish_display_path_message(int goal_idx, const std::vector<tree_node>& nodes);
public:

    RRTPlanner();
    RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

                void people_callback(const people_msgs::People::ConstPtr& msg);
};

} // namespace rrt_planner
 #endif
