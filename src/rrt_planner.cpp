#include "rrt_planner/rrt_planner.h"

#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_planner {

using namespace std;
using costmap_2d::FREE_SPACE;

RRTPlanner::RRTPlanner() : pn("~") {

}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : pn("~") {
    initialize(name, costmap_ros);
}

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        tree_pub = pn.advertise<visualization_msgs::Marker>("/rrt_paths", 10);
        people_sub = pn.subscribe("/people", 1000, &RRTPlanner::people_callback, this);

        pn.param<double>("/move_base/DWAPlannerROS/max_vel_x", mean_speed, 0.55);

        increment_dist = 0.6;
        min_turn_radius = 0.3;
        travel_cost = 1.0;

        costmap_ros_ = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
        costmap_ = costmap_ros_->getCostmap(); //get the costmap_ from costmap_ros_
        footprint = costmap_ros_->getRobotFootprint();

        // initialize other planner parameters
        /*ros::NodeHandle private_nh("~/" + name);
        private_nh.param("step_size", step_size_, costmap_->getResolution());
        private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);*/
        world_model_ = new base_local_planner::CostmapModel(*costmap_);

        initialized_ = true;
    }
    else {
        ROS_WARN("This planner has already been initialized... doing nothing");
    }
}

void RRTPlanner::people_callback(const people_msgs::People::ConstPtr& msg)
{
    people = msg->people;
}

double RRTPlanner::cost(const geometry_msgs::Pose& pose)
{
    tf::Stamped<tf::Pose> tf_pose;
    poseMsgToTF(pose, tf_pose);
    double useless_pitch, useless_roll, yaw;
    tf_pose.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);
    return world_model_->footprintCost(pose.position.x, pose.position.y, yaw, footprint);
}

geometry_msgs::Pose RRTPlanner::sample()
{
    // get min/max of costmap
    double maxx = costmap_->getSizeInMetersX() - costmap_->getOriginX();
    double minx = costmap_->getOriginX();
    double maxy = costmap_->getSizeInMetersY() - costmap_->getOriginY();
    double miny = costmap_->getOriginY();

    unsigned char* grid = costmap_->getCharMap();

    // sample until in free space
    double randx, randy, randtheta;
    while (true) {
        randx = double(rand())/double(RAND_MAX)*(maxx - minx) + minx;
        randy = double(rand())/double(RAND_MAX)*(maxy - miny) + miny;
        randtheta = double(rand())/double(RAND_MAX)*2.0*M_PI; // - M_PI?

        unsigned int gridx, gridy;
        if (!costmap_->worldToMap(randx, randy, gridx, gridy)) {
            continue;
        }
        int index = costmap_->getIndex(gridx, gridy);

        if (grid[index] != FREE_SPACE) { // can't go here
            continue;
        }
        double randcost = world_model_->footprintCost(randx, randy, randtheta, footprint);
        if (randcost >= 0) {
            break;
        }
    }

    geometry_msgs::Pose new_goal;
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(randtheta);
    new_goal.position.x = randx;
    new_goal.position.y = randy;
    new_goal.orientation.x = goal_quat.x();
    new_goal.orientation.y = goal_quat.y();
    new_goal.orientation.z = goal_quat.z();
    new_goal.orientation.w = goal_quat.w();

    return new_goal;
}

double pose_squared_dist(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
    const double xdiff = p1.position.x-p2.position.x;
    const double ydiff = p1.position.y-p2.position.y;
    return xdiff*xdiff+ydiff*ydiff;
}

double pose_angle_dist(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
    return fabs(acos(p1.position.x*p2.position.x+p1.position.y*p2.position.y));
}

int RRTPlanner::nearest(const geometry_msgs::Pose& pose, pcl::octree::OctreePointCloudSearch<PointT>& octree)
{
    vector<int> idxs;
    vector<float> sqrd_dists;
    PointT pose_xy { pose.position.x, pose.position.y, 0.0f };
    int found = octree.nearestKSearch(pose_xy, 1, idxs, sqrd_dists);
    return idxs[0];
}

vector<int> RRTPlanner::near(const geometry_msgs::Pose& pose, const pcl::octree::OctreePointCloudSearch<PointT>& octree)
{
    const float radius = 1.0f;

    PointT pose_xy { pose.position.x, pose.position.y, 0.0f };
    vector<int> idxs;
    vector<float> sqrd_dists;
    int found = octree.radiusSearch(pose_xy, radius, idxs, sqrd_dists);
    return idxs;
}

//pair<geometry_msgs::Pose, geometry_msgs::Point>
pair<geometry_msgs::Pose, double> RRTPlanner::steer_towards(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
    tf::Stamped<tf::Pose> tf_pose;
    poseMsgToTF(p1, tf_pose);
    double useless_pitch, useless_roll, yaw;
    tf_pose.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

    Eigen::Vector2d point(p1.position.x, p1.position.y);
    Eigen::Vector2d diff(point(0)-p2.position.x, point(1)-p2.position.y);
    Eigen::Vector2d normal(-sin(yaw), cos(yaw));
    double radius = -0.5*diff.squaredNorm()/diff.dot(normal);
    if (fabs(radius) < min_turn_radius) {
        radius = copysign(min_turn_radius, radius);
    }
    Eigen::Vector2d center = point + radius*normal;

    Eigen::Vector2d on_circle1 = point - center;
    double theta = atan2(on_circle1(1), on_circle1(0)); // same as yaw????

    double thetaprim = theta + increment_dist/radius;
    Eigen::Vector2d target = center + fabs(radius)*Eigen::Vector2d(cos(thetaprim), sin(thetaprim));

    geometry_msgs::Pose rtn = p1;
    rtn.position.x = target(0);
    rtn.position.y = target(1);
    if (radius < 0) {
        thetaprim -= M_PI/2.0;
    }
    else {
        thetaprim += M_PI/2.0;
    }
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(thetaprim);
    rtn.orientation.x = goal_quat.x();
    rtn.orientation.y = goal_quat.y();
    rtn.orientation.z = goal_quat.z();
    rtn.orientation.w = goal_quat.w();

    return make_pair(rtn, increment_dist);
}

pair<geometry_msgs::Pose, double> RRTPlanner::steer(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
    tf::Stamped<tf::Pose> tf_pose;
    poseMsgToTF(p1, tf_pose);
    double useless_pitch, useless_roll, yaw;
    tf_pose.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

    Eigen::Vector2d point(p1.position.x, p1.position.y);
    Eigen::Vector2d diff(point(0)-p2.position.x, point(1)-p2.position.y);
    Eigen::Vector2d normal(-sin(yaw), cos(yaw));
    double radius = -0.5*diff.squaredNorm()/diff.dot(normal);

    if (fabs(radius) < min_turn_radius) {
        return make_pair(p1, -1.0);
    }
    Eigen::Vector2d center = point + radius*normal;

    Eigen::Vector2d on_circle1 = point - center;
    double theta1 = atan2(on_circle1(1), on_circle1(0)); // same as yaw????

    Eigen::Vector2d on_circle2 = Eigen::Vector2d(p2.position.x, p2.position.y) - center;
    double theta2 = atan2(on_circle2(1), on_circle2(0)); // same as yaw????

    // fmod 2*M_PI
    double arc_length = fmod(fabs(theta1 - theta2), 2.0*M_PI)*fabs(radius);

    geometry_msgs::Pose rtn = p2;
    if (radius < 0) {
        theta2 -= M_PI/2.0;
    }
    else {
        theta2 += M_PI/2.0;
    }
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(theta2);
    rtn.orientation.x = goal_quat.x();
    rtn.orientation.y = goal_quat.y();
    rtn.orientation.z = goal_quat.z();
    rtn.orientation.w = goal_quat.w();

    return make_pair(rtn, arc_length);
}

vector<geometry_msgs::Point> RRTPlanner::generate_local_path(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
    tf::Stamped<tf::Pose> tf_pose;
    poseMsgToTF(p1, tf_pose);
    double useless_pitch, useless_roll, yaw;
    tf_pose.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

    Eigen::Vector2d point(p1.position.x, p1.position.y);
    Eigen::Vector2d diff(point(0)-p2.position.x, point(1)-p2.position.y);
    Eigen::Vector2d normal(-sin(yaw), cos(yaw));
    double radius = -0.5*diff.squaredNorm()/diff.dot(normal);

    Eigen::Vector2d center = point + radius*normal;

    Eigen::Vector2d on_circle1 = point - center;
    double theta1 = atan2(on_circle1(1), on_circle1(0)); // same as yaw????

    Eigen::Vector2d on_circle2 = Eigen::Vector2d(p2.position.x, p2.position.y) - center;
    double theta2 = atan2(on_circle2(1), on_circle2(0)); // same as yaw????

    if (theta2-theta1 > M_PI) {
        theta2 -= 2.0*M_PI;
    }
    else if (theta2-theta1 < -M_PI) {
        theta2 += 2.0*M_PI;
    }
    double increment = copysign(0.05/fabs(radius), theta2-theta1);

    vector<geometry_msgs::Point> path;
    for (double theta = theta1; fabs(theta-theta2) > 0.05/fabs(radius); theta += increment) {
        geometry_msgs::Point point;
        point.x = center(0)+fabs(radius)*cos(theta);
        point.y = center(1)+fabs(radius)*sin(theta);
        point.z = 0.0f;
        path.push_back(point);
    }

    return path;
}

void RRTPlanner::publish_display_tree_message(const vector<tree_node>& nodes)
{
    Eigen::Quaterniond quat;
    quat.setIdentity();

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace"; // what's this for?
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    // add a line strip between all nodes and parent nodes
    // later add the archs also
    marker.points.resize(2*nodes.size()-2);
    int idx = 0;
    for (const tree_node& t : nodes) {
        if (t.parent_idx == -1) {
            continue;
        }
        marker.points[idx] = t.pose.position;
        marker.points[idx+1] = nodes[t.parent_idx].pose.position;
        idx += 2;
    }

    tree_pub.publish(marker);
}

void RRTPlanner::publish_display_path_message(int goal_idx, const vector<tree_node>& nodes)
{
    Eigen::Quaterniond quat;
    quat.setIdentity();

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace"; // what's this for?
    marker.id = 1;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
    marker.scale.x = 0.04;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    int next_idx = nodes[goal_idx].parent_idx;
    geometry_msgs::Pose previous_pose = nodes[goal_idx].pose;
    while (next_idx != -1) {
        geometry_msgs::Pose current_pose = nodes[next_idx].pose;
        vector<geometry_msgs::Point> local_path = generate_local_path(current_pose, previous_pose);
        marker.points.insert(marker.points.end(), local_path.rbegin(), local_path.rend());
        next_idx = nodes[next_idx].parent_idx;
        previous_pose = current_pose;
    }

    tree_pub.publish(marker);
}

tree_node RRTPlanner::choose_parent(const geometry_msgs::Pose& towards_sampled, vector<int>& T,
                                    int initial_parent, vector<tree_node>& nodes)
{

    double mincost = nodes[initial_parent].accum_cost + travel_cost*increment_dist;
    double minidx = initial_parent;
    geometry_msgs::Pose minpose = towards_sampled;

    for (int idx : T) {
        if (idx == initial_parent) {
            continue;
        }
        geometry_msgs::Pose towards_pose;
        double dist;
        tie(towards_pose, dist) = steer(nodes[idx].pose, towards_sampled);
        if (dist < 0) {
            continue;
        }
        double cost = nodes[idx].accum_cost + travel_cost*dist;
        if (cost < mincost) {
            mincost = cost;
            minidx = idx;
            minpose = towards_pose;
        }
    }

    tree_node new_node { minpose, mincost, minidx };
    return new_node;
}

void RRTPlanner::rewire(const tree_node& new_node, int new_ind, vector<int>& T,
                        vector<tree_node>& nodes)
{
    for (int idx : T) {
        if (idx == new_node.parent_idx) {
            continue;
        }
        double dist;
        geometry_msgs::Pose pose;
        tie(pose, dist) = steer(new_node.pose, nodes[idx].pose);
        // also check if angle disance between poses
        // is too big
        if (dist < 0 || pose_angle_dist(pose, nodes[idx].pose) > M_PI/10.0) {
            continue;
        }
        double new_cost = travel_cost*dist + new_node.accum_cost;
        if (new_cost < nodes[idx].accum_cost) {
            nodes[idx].parent_idx = new_ind;
            nodes[idx].accum_cost = new_cost;
        }
    }
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan )
{
    costmap_ = costmap_ros_->getCostmap();
    footprint = costmap_ros_->getRobotFootprint();

    plan.clear();
    last_people = people;

    vector<tree_node> nodes;
    tree_node root {start.pose, 0.0, -1};
    nodes.push_back(root);

    PointT start_xy { start.pose.position.x, start.pose.position.y, 0.0f };
    CloudT::Ptr cloud(new CloudT);
    cloud->push_back(start_xy);
    float resolution = 0.4;
    pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    int minidx = -1;
    double mincost = std::numeric_limits<double>::infinity();
    const int N = 10000;
    for (int i = 0; i < N; ++i) {
        geometry_msgs::Pose sampled = sample();
        // find closest node
        int nearest_idx = nearest(sampled, octree);
        tree_node nearest = nodes[nearest_idx];
        geometry_msgs::Pose towards_sampled;
        double dist;
        tie(towards_sampled, dist) = steer_towards(nearest.pose, sampled);
        double towards_cost = cost(towards_sampled);
        if (towards_cost < 0) {
            continue;
        }
        vector<int> T = near(towards_sampled, octree);
        // we might also need the resulting angle and the new cost here

        tree_node new_node;
        new_node = choose_parent(towards_sampled, T, nearest_idx, nodes);

        // now check which ones are nearby, pick the closest
        nodes.push_back(new_node);
        PointT new_xy { new_node.pose.position.x, new_node.pose.position.y, 0.0f };
        octree.addPointToCloud(new_xy, cloud);

        rewire(new_node, nodes.size()-1, T, nodes);

        // we actually have to check this for a lot more nodes now
        if (pose_squared_dist(goal.pose, new_node.pose) < 0.2*0.2 &&
            new_node.accum_cost < mincost) {
            mincost = new_node.accum_cost;
            minidx = nodes.size() - 1;
        }
    }

    cout << "Min path cost: " << mincost << endl;

    publish_display_tree_message(nodes);

    if (minidx == -1) {
        return false;
    }

    publish_display_path_message(minidx, nodes);

    int next_idx = minidx;
    plan.push_back(goal);
    while (next_idx != -1) {
        geometry_msgs::PoseStamped pose = start;
        pose.pose = nodes[next_idx].pose;
        plan.push_back(pose);
        next_idx = nodes[next_idx].parent_idx;
    }
    plan.push_back(start);

    std::reverse(plan.begin(), plan.end());

    return true;
}

}
