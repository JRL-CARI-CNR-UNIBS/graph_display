# This Python file uses the following encoding: utf-8
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  config_folder = PathJoinSubstitution([FindPackageShare("graph_plot"),"config"])

  return LaunchDescription([
    ExecuteProcess(
        cmd = [
          FindExecutable(name="cnr_param_server"),
          "--path-to-file",
          PathJoinSubstitution([
            config_folder,
            "test_ik.yaml.template" 
          ])
        ],
        shell=False
      ),

    Node(
      package="graph_plot",
      executable="plot_ik",
      output="screen",
      namespace="plot_ik",
      ros_arguments=["--log-level", "info"]
      )
])