#include <ros/ros.h>
#include <graph_core/graph/path.h>
#include <graph_core/collision_checkers/cube_3d_collision_checker.h>
#include <graph_display/graph_display.h>
#include <graph_core/metrics/euclidean_metrics.h>

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
    ROS_ERROR("%s/group_name is not defined",pnh.getNamespace().c_str());
    return 0;
  }

  std::string package_name = "graph_plot";
  std::string package_path = ros::package::getPath(package_name);

  if (package_path.empty())
  {
    ROS_ERROR_STREAM("Failed to get path for package '" << package_name);
    return 0;
  }

  std::string logger_file = package_path+"/config/logger_param.yaml";
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("plot_tree",logger_file);


  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  graph::core::CollisionCheckerPtr checker = std::make_shared<graph::core::Cube3dCollisionChecker>(logger);

  graph::core::MetricsPtr metrics=std::make_shared<graph::core::EuclideanMetrics>(logger);

  std::string what;


  graph::display::Display display_path(planning_scene,group_name);
  display_path.clearMarkers();

  YAML::Node config;

  try {
    config = YAML::LoadFile(package_path+"/config/trees.yaml");
  } catch (const YAML::Exception& e) {
    ROS_ERROR_STREAM("Error loading YAML file: "<<e.what());
    return 0;
  }

  std::vector<std::string> tree_names;
  if (!pnh.getParam("plot_tree_names",tree_names))
  {
    ROS_ERROR("%s/plot_tree_name is not defined",pnh.getNamespace().c_str());
    return 0;
  }

  ros::Rate lp(1);
  while (ros::ok())
  {
    for (std::string& tree_name: tree_names)
    {
      if (!ros::ok())
        break;

      YAML::Node p;
      if (config[tree_name])
        p = config[tree_name];
      else
      {
        ROS_ERROR_STREAM("Unable to load "<<tree_name);
        return 0;
      }

      graph::core::TreePtr tree=graph::core::Tree::fromYAML(p,metrics,checker,logger);
      if (tree)
      {
        tree->recheckCollision();
        display_path.clearMarkers();
        display_path.displayTree(tree);

        lp.sleep();
      }
    }
  }


  ros::spin();

  return 0;
}
