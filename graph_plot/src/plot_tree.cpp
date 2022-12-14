#include <ros/ros.h>
#include <graph_core/graph/path.h>
#include <graph_core/moveit_collision_checker.h>
#include <graph_core/graph/graph_display.h>

#include <rosparam_utilities/rosparam_utilities.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plot_tree");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string group_name;
  if (!pnh.getParam("group_name",group_name))
  {
    ROS_ERROR("%s/move_group is not defined",pnh.getNamespace().c_str());
    return 0;
  }

  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name);

  pathplan::MetricsPtr metrics=std::make_shared<pathplan::Metrics>();

  std::string what;


  pathplan::Display display_path(planning_scene,group_name);
  display_path.clearMarkers();

  std::string tree_name="tree";
  if (!nh.getParam("plot_tree_name",tree_name))
  {
    ROS_ERROR("%s/plot_tree_name is not defined",pnh.getNamespace().c_str());
    return 0;
  }
  double maximum_distance=0.1;
  XmlRpc::XmlRpcValue p;
  if (!pnh.getParam(tree_name,p))
  {
    ROS_ERROR("%s is unable to load trees",pnh.getNamespace().c_str());
    return 0;
  }
  pathplan::TreePtr tree=pathplan::Tree::fromXmlRpcValue(p,maximum_distance,checker,metrics,true);

  display_path.displayTree(tree);


  ros::spin();

  return 0;
}
