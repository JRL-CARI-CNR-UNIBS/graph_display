#include <ros/ros.h>
#include <graph_core/graph/path.h>
#include <graph_core/cube_3d_collision_checker.h>
#include <graph_display/graph_display.h>

#include <rosparam_utilities/rosparam_utilities.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "plot_paths");
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  std::string group_name;
  if (!pnh.getParam("group_name",group_name))
  {
    ROS_ERROR("group_name is not defined");
    return 0;
  }

  std::vector<std::string> pose_names;
  if (!pnh.getParam("pose_names",pose_names))
  {
    ROS_ERROR("pose_names is not defined");
    return 0;
  }

  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  std::string what;

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  moveit_msgs::RobotState state_msg;
  moveit_msgs::DisplayRobotState display_state_msg;


  std::vector<ros::Publisher> pubs(pose_names.size());
  std::vector<std::vector<std::vector<double>>> ik_solutions(pose_names.size());

  for (unsigned int ip=0;ip<pose_names.size();ip++)
  {
    XmlRpc::XmlRpcValue param;
    if (!pnh.getParam(pose_names.at(ip),param))
    {
      ROS_ERROR("%s is not defined",pose_names.at(ip).c_str());
      continue;
    }
    ros::Publisher state_pub=nh.advertise<moveit_msgs::DisplayRobotState>(pose_names.at(ip),1);

    if (param.getType() != XmlRpc::XmlRpcValue::Type::TypeArray)
    {
      ROS_ERROR("%s is not not an array",pose_names.at(ip).c_str());
      continue;
    }
    std::vector<std::vector<double>> solutions(param.size());
    for (int idx=0;idx<param.size();idx++)
    {
      if (param[idx].getType() != XmlRpc::XmlRpcValue::Type::TypeArray)
      {
        ROS_ERROR("%s is not not an array of arrays",pose_names.at(ip).c_str());
        return 0;
      }
      solutions.at(idx).resize(param[idx].size());
      for (int iel=0;iel<param[idx].size();iel++)
        solutions.at(idx).at(iel)=param[idx][iel];
    }
    ik_solutions.at(ip)=solutions;
    pubs.at(ip)=state_pub;
  }



  while (ros::ok())
  {
    for (size_t ip=0;ip<pose_names.size();ip++)
    {
      for (size_t idx=0;idx<ik_solutions.at(ip).size();idx++)
      {
        kinematic_state->setJointGroupPositions(group_name, ik_solutions.at(ip).at(idx));

        moveit::core::robotStateToRobotStateMsg(*kinematic_state,state_msg);
        display_state_msg.state=state_msg;

        pubs.at(ip).publish(display_state_msg);
        ros::Duration(1).sleep();
      }
    }
  }
  return 0;
}
