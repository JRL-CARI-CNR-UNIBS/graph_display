#include <ros/ros.h>
#include <graph_core/graph/path.h>
#include <graph_core/moveit_collision_checker.h>
#include <graph_core/graph/graph_display.h>

#include <rosparam_utilities/rosparam_utilities.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plot_paths");

  ros::NodeHandle pnh("~");

  std::string group_name;
  if (!pnh.getParam("group_name", group_name))
  {
    ROS_ERROR("%s/group_name is not defined", pnh.getNamespace().c_str());
    return 0;
  }

  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  pathplan::CollisionCheckerPtr checker =
      std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name);

  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();

  std::string what;

  pathplan::Display display_path(planning_scene, group_name);
  display_path.clearMarkers();

  std::vector<std::string> path_names;
  if (!pnh.getParam("display_path", path_names))
  {
    ROS_ERROR("display_path is not defined");
    return 0;
  }

  std::vector<pathplan::PathPtr> paths;
  std::vector<int> path_ids;
  std::vector<std::vector<double>> path_colors;

  std::vector<double> color;
  color = { 1, 0, 0, 1 };
  for (std::string path_name : path_names)
  {
    std::vector<Eigen::VectorXd> waypoints;

    if (!rosparam_utilities::getParam(pnh, path_name, waypoints, what))
    {
      ROS_DEBUG("Path %s is not correct.", path_name.c_str());
      continue;
    }
    if (waypoints.size() <= 1)
      continue;

    if (!pnh.getParam(path_name + "/color", color))
    {
      ROS_WARN("path %s has no color, using red", path_name.c_str());
      double tmp = color.at(0);
      color.at(0) = color.at(1);
      color.at(1) = color.at(2);
      color.at(2) = tmp;
    }
    assert(color.size() == 4);
    path_colors.push_back(color);

    pathplan::NodePtr n1 = std::make_shared<pathplan::Node>(waypoints.at(0));
    std::vector<pathplan::ConnectionPtr> conns;
    for (size_t idx = 1; idx < waypoints.size(); idx++)
    {
      pathplan::NodePtr n2 = std::make_shared<pathplan::Node>(waypoints.at(idx));
      pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(n1, n2);
      conns.push_back(conn);
      n1 = n2;
    }

    pathplan::PathPtr path = std::make_shared<pathplan::Path>(conns, metrics, checker);
    paths.push_back(path);
    path_ids.push_back(display_path.displayPath(path, paths.size()));
  }

  ros::Rate lp(1);
  while (ros::ok())
  {
    for (size_t idx = 0; idx < paths.size(); idx++)
    {
      path_ids.at(idx) = display_path.displayPath(paths.at(idx), path_ids.at(idx), "pathplan", path_colors.at(idx));
    }
    lp.sleep();
  }
  ros::spin();

  return 0;
}
