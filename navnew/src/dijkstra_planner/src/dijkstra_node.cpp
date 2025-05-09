#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "dijkstra_planner/dijkstra.hpp"  // Your custom planner header

using namespace dijkstra_planner;

class DijkstraNode
{
public:
    DijkstraNode() : nh_("~"), start_received_(false), goal_received_(false)
    {
        ROS_INFO("Initializing DijkstraNode...");

        // Get grid dimensions from parameter server or use defaults
        int width, height;
        nh_.param("grid_width", width, 10);
        nh_.param("grid_height", height, 10);

        // Initialize the planner
        planner_ = std::make_shared<DijkstraPlanner>(width, height);
        ROS_INFO("Dijkstra Planner Initialized with grid size: [%d x %d]", width, height);

        // Set up subscriptions and publisher
        start_sub_ = nh_.subscribe("/start", 1, &DijkstraNode::startCallback, this);
        goal_sub_ = nh_.subscribe("/goal", 1, &DijkstraNode::goalCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/dijkstra_path", 1);

        ROS_INFO("✅ Dijkstra Node Initialized and Ready");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber start_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;
    geometry_msgs::Pose start_;
    geometry_msgs::Pose goal_;
    bool start_received_;
    bool goal_received_;
    std::shared_ptr<DijkstraPlanner> planner_;

    void startCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        start_ = msg->pose;
        start_received_ = true;
        ROS_INFO("📍 Start pose received: [%.2f, %.2f]", start_.position.x, start_.position.y);
        ROS_INFO("Start callback executed successfully.");
        tryComputePath();
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        goal_ = msg->pose;
        goal_received_ = true;
        ROS_INFO("🎯 Goal pose received: [%.2f, %.2f]", goal_.position.x, goal_.position.y);
        ROS_INFO("Goal callback executed successfully.");
        tryComputePath();
    }

    void tryComputePath()
    {
        if (!start_received_ || !goal_received_)
        {
            ROS_WARN("⚠️ Waiting for both start and goal to be received...");
            return;
        }

        ROS_INFO("Start and Goal received, computing path...");

        std::vector<geometry_msgs::Pose> path = planner_->calculateDijkstraPath(start_, goal_);
        if (path.empty())
        {
            ROS_WARN("No valid path found.");
            return;
        }

        // Prepare the path for publishing
        nav_msgs::Path ros_path;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = "map";

        for (const auto& pose : path)
        {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose = pose;
            pose_stamped.header = ros_path.header;
            ros_path.poses.push_back(pose_stamped);
        }

        // Publish the path
        path_pub_.publish(ros_path);
        ROS_INFO("✅ Published Dijkstra path with %lu poses", ros_path.poses.size());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dijkstra_node");
    ROS_INFO("ROS node initialized: dijkstra_node");

    DijkstraNode node;
    ros::spin();  // Keep the node running
    return 0;
}

