#include <graph_core/util.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class PlotIk : public rclcpp::Node
{
public:
  PlotIk(const cnr_logger::TraceLoggerPtr& logger,
         const std::string group_name,
         const std::vector<std::string>& pose_names,
         const rclcpp::NodeOptions & node_options)
    : Node("plot_ik", node_options), group_name_(group_name), logger_(logger), pose_names_(pose_names)
  {
    pubs_.resize(pose_names_.size());
    ik_solutions_.resize(pose_names_.size());

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

  void plot_ik()
  {
    moveit_msgs::msg::RobotState state_msg;
    moveit_msgs::msg::DisplayRobotState display_state_msg;
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model_));

    while (rclcpp::ok())
    {
      for (size_t ip=0;ip<pose_names_.size();ip++)
      {
        for (size_t idx=0;idx<ik_solutions_.at(ip).size();idx++)
        {

          kinematic_state->setJointGroupPositions(group_name_,ik_solutions_.at(ip).at(idx));

          moveit::core::robotStateToRobotStateMsg(*kinematic_state,state_msg);
          display_state_msg.state=state_msg;

          pubs_[ip]->publish(display_state_msg);
          rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }
      }
    }
  }

private:
  std::string group_name_;
  cnr_logger::TraceLoggerPtr logger_;
  std::vector<std::string> pose_names_;
  moveit::core::RobotModelPtr kinematic_model_;
  std::vector<std::vector<std::vector<double>>> ik_solutions_;
  std::vector<rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr> pubs_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Load logger configuration file
  std::string package_name = "graph_plot";
  std::string package_path = ament_index_cpp::get_package_share_directory(package_name);

  if (package_path.empty())
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("plot_ik"),"Failed to get path for package '" << package_name);
    return 1;
  }

  std::string logger_file = package_path+"/config/logger_param.yaml";
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("plot_ik",logger_file);

  // Get the robot description
  std::string param_ns = "";
  std::string group_name;
  if(not graph::core::get_param(logger,param_ns,"group_name",group_name))
    return 1;

  std::vector<std::string> pose_names;
  if(not graph::core::get_param(logger,param_ns,"pose_names",pose_names))
    return 1;

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<PlotIk>(logger,group_name,pose_names,node_options);

  node->plot_ik();

  //  rclcpp::executors::MultiThreadedExecutor executor;
  //  executor.add_node(node);
  //  executor.spin();

  return 0;
}
