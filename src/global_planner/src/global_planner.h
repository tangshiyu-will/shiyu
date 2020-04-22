 /** include the libraries you need in your planner here */
 /** for global path planner interface */
 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>

 using std::string;

 #ifndef GLOBAL_PLANNER_CPP
 #define GLOBAL_PLANNER_CPP

 namespace global_planner {

 class GlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:

  GlobalPlanner();
  GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**   **/
  /**void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan
               );
  };
 };
 #endif */
   void CarrotPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
     if(!initialized_){
       costmap_ros_ = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
       costmap_ = costmap_ros_->getCostmap(); //get the costmap_ from costmap_ros_

      // initialize other planner parameters
       ros::NodeHandle private_nh("~/" + name);
       private_nh.param("step_size", step_size_, costmap_->getResolution());
       private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
       world_model_ = new base_local_planner::CostmapModel(*costmap_);

       initialized_ = true;
     }
     else
       ROS_WARN("This planner has already been initialized... doing nothing");
   }
