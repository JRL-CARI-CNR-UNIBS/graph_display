#include <ros/ros.h>
#include <graph_core/graph/path.h>
#include <graph_core/cube_3d_collision_checker.h>
#include <graph_display/graph_display.h>
#include <cnr_logger/cnr_logger.h>

#include <rosparam_utilities/rosparam_utilities.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <std_srvs/Trigger.h>
#include "ros/service_server.h"
#include "std_srvs/TriggerRequest.h"
#include "std_srvs/TriggerResponse.h"

#include <thread>

std::thread loop;

void loop_animation()
{
  ros::NodeHandle pnh("~");

  std::string group_name;
  if (!pnh.getParam("group_name", group_name))
  {
    ROS_ERROR("%s/group_name is not defined", pnh.getNamespace().c_str());
    return;
  }

  std::string package_name = "graph_plot";
  std::string package_path = ros::package::getPath(package_name);

  if (package_path.empty())
  {
    ROS_ERROR_STREAM("Failed to get path for package '" << package_name);
    return;
  }

  std::string logger_file = package_path+"/config/logger_param.yaml";
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("plot_paths",logger_file);

  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  graph::core::CollisionCheckerPtr checker =
      std::make_shared<graph::core::Cube3dCollisionChecker>(logger);

  graph::core::MetricsPtr metrics = std::make_shared<graph::core::Metrics>(logger);

  std::string what;

  graph::display::Display display_path(planning_scene, group_name);
  display_path.clearMarkers();

  std::vector<std::string> path_names;
  if (!pnh.getParam("display_path", path_names))
  {
    ROS_ERROR("display_path is not defined");
    return;
  }

  std::vector<graph::core::PathPtr> paths;
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

    graph::core::NodePtr n1 = std::make_shared<graph::core::Node>(waypoints.at(0));
    std::vector<graph::core::ConnectionPtr> conns;
    for (size_t idx = 1; idx < waypoints.size(); idx++)
    {
      graph::core::NodePtr n2 = std::make_shared<graph::core::Node>(waypoints.at(idx));
      graph::core::ConnectionPtr conn = std::make_shared<graph::core::Connection>(n1, n2, logger);
      conns.push_back(conn);
      n1 = n2;
    }

    graph::core::PathPtr path = std::make_shared<graph::core::Path>(conns, metrics, checker, logger);
    paths.push_back(path);
    path_ids.push_back(display_path.displayPath(path, paths.size()));
  }

  ros::Rate lp(1);
  while (ros::ok())
  {
    for (size_t idx = 0; idx < paths.size(); idx++)
    {
      path_ids.at(idx) = display_path.displayPath(paths.at(idx), path_ids.at(idx), "graph_display", path_colors.at(idx));
    }
    lp.sleep();
  }
}

bool start_display_the_path_markers(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& res)
{
  ROS_INFO("Start Display The Markers of the Path!");

  ROS_INFO(".. starting the thread");
  loop = std::thread(loop_animation);
  loop.detach();

  res.success = true;
  ROS_INFO("Ok, to kill the thread, just ctrl+c");
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plot_paths");

  ros::NodeHandle pnh("~");

  ros::ServiceServer plot_markers = pnh.advertiseService("/start_markers_path", &start_display_the_path_markers);

  ros::spin();

  return 0;  
}
