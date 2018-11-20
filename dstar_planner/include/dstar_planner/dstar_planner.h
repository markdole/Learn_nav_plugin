//
// Created by ma on 18-11-17.
//

#ifndef DSTAR_PLANNER_DSTAR_PLANNER_H
#define DSTAR_PLANNER_DSTAR_PLANNER_H
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

namespace dstar_planner {

    class DstarPlanner : public nav_core::BaseGlobalPlanner {
    public:
        DstarPlanner();
        /**
       * @brief  Constructor for the CarrotPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
        DstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /**override classes from interface nav_core:: BaseGlobalPlanner **/
        void initialize (std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        /**  inintialize function for dstar planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         * **/
        bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
        /**
        * @brief Given a goal pose in the world, compute a plan
        * @param start The start pose
        * @param goal The goal pose
        * @param plan The plan... filled by the planner
         *       ^^^^this is what we gonna design^^^^^^^
        * @return True if a valid plan was found, false otherwise
        */
    };
}

#endif //DSTAR_PLANNER_DSTAR_PLANNER_H
