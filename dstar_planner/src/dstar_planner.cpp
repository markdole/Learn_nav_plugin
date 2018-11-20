//
// Created by ma on 18-11-17.
//

#include <dstar_planner/dstar_planner.h>
#include <pluginlib/class_list_macros.h>

/** register the plugin**/
PLUGINLIB_EXPORT_CLASS( dstar_planner::DstarPlanner, nav_core::BaseGlobalPlanner)
//using namespace std;
//default Constructor
namespace dstar_planner{

    DstarPlanner::DstarPlanner(){}

    DstarPlanner::DstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }

    void DstarPlanner::initialize (std::string name, costmap_2d::Costmap2DROS* costmap_ros){}

    bool DstarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
            std::vector<geometry_msgs::PoseStamped>& plan){
        plan.push_back(start);
        geometry_msgs::PoseStamped distance;
        distance.pose.position.x = goal.pose.position.x - start.pose.position.x;
        distance.pose.position.y = goal.pose.position.y - start.pose.position.y;
        for (int i = 0; i < 20; i++) {
            geometry_msgs::PoseStamped new_goal = goal;
            tf::Quaternion goal_quat = tf::createQuaternionFromYaw(0);

            new_goal.pose.position.x =start.pose.position.x +(0.05 * i)*distance.pose.position.x;
            new_goal.pose.position.y =start.pose.position.y +(0.05 * i)*distance.pose.position.y;

            new_goal.pose.orientation.x = goal_quat.x();
            new_goal.pose.orientation.y = goal_quat.y();
            new_goal.pose.orientation.z = goal_quat.z();
            new_goal.pose.orientation.w = goal_quat.w();

            plan.push_back(new_goal);
        }
        plan.push_back(goal);
        return true;
    }
};
