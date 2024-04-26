#include <graph_core/graph/path.h>
#include <graph_core/collision_checkers/cube_3d_collision_checker.h>
#include <graph_display/graph_display.h>
#include <cnr_logger/cnr_logger.h>
#include <graph_core/metrics/euclidean_metrics.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <std_srvs/srv/trigger.hpp>

#include <thread>

std::thread loop;

class PlotPaths : public rclcpp::Node
{
public:
  PlotPaths(const cnr_logger::TraceLoggerPtr& logger,
         const std::string group_name,
         const std::vector<std::string>& pose_names,
         const rclcpp::NodeOptions & node_options)
    : Node("plot_paths", node_options), group_name_(group_name), logger_(logger), pose_names_(pose_names)
  {

    // Read ik
    for (unsigned int ip=0;ip<pose_names_.size();ip++)
    {
      if(not graph::core::get_param(logger,"",pose_names_[ip],ik_solutions_[ip]))
      {
        CNR_ERROR(logger_,"Cannot get "<<pose_names_[ip]);
        throw std::runtime_error("cannot get ik");
      }
      pubs_[ip]=this->create_publisher<moveit_msgs::msg::DisplayRobotState>(pose_names_.at(ip),1);
    }

    moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(),group_name_);
    robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this(),"robot_description");
    kinematic_model_ = robot_model_loader.getModel();
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


private:
  std::string group_name_;
  cnr_logger::TraceLoggerPtr logger_;
  moveit::core::RobotModelPtr kinematic_model_;
  std::vector<rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr> pubs_;

  void loop_animation()
  {
    planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model_);
    graph::core::CollisionCheckerPtr checker =
        std::make_shared<graph::core::Cube3dCollisionChecker>(logger_);

    graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger_);
    graph::display::Display display_path(planning_scene, group_name_);

    display_path.clearMarkers();

    std::vector<std::string> path_names;
    if(!graph::core::get_param(logger_,"","display_path",path_names))
    {
      CNR_ERROR(logger_,"display_path is not defined");
      return 1;
    }

    std::vector<int> path_ids;
    std::vector<graph::core::PathPtr> paths;
    std::vector<std::vector<double>> path_colors;

    std::vector<double> color;
    color = { 1, 0, 0, 1 };
    for (std::string path_name : path_names)
    {
      std::vector<Eigen::VectorXd> waypoints;

      //get YAML::Node of the path from cnr_param
  //    if (!rosparam_utilities::getParam(pnh, path_name, waypoints, what)) FIX
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
};



void loop_animation2()
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

  graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);

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

//    if (!rosparam_utilities::getParam(pnh, path_name, waypoints, what)) FIX
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

bool start_display_the_path_markers2(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& res)
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
