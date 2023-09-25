#!/usr/bin/env python3
import rospy
import moveit_msgs.msg
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

if __name__ == "__main__":
    rospy.init_node('get_ik')

    display_path = rospy.get_param("~display_path")
    joint_name = rospy.get_param("joint_names")
    state = RobotState()
    state.joint_state.name = joint_name
    state.joint_state.velocity = [0] * len(joint_name)
    state.joint_state.effort = [0] * len(joint_name)

    speed = rospy.get_param("~speed",1)
    pubs = dict()

    for path in display_path:
        pubs[path] = rospy.Publisher(path, DisplayTrajectory, queue_size=10, latch=True)

        try:
            points = rospy.get_param("~"+path)
        except:
            print(f"problems loading path {path}")
            continue
        first_time = True  # true if first point
        print(f"point:\n{points}")
        trj = RobotTrajectory()
        trj.joint_trajectory.joint_names = joint_name
        trj.joint_trajectory.header.frame_id="world"
        for c in points:
            print(f"point: {c}")
            if first_time:
                first_time = False
                last_c = c
                state.joint_state.position = c
            distance = rospy.Duration.from_sec(max([abs(j - i) for i, j in zip(c, last_c)])/speed)

            pnt = JointTrajectoryPoint()
            pnt.effort = [0] * len(joint_name)
            pnt.velocities = [0] * len(joint_name)
            pnt.positions = c
            pnt.time_from_start = 0.5 * distance

            trj.joint_trajectory.points.append(pnt)

            last_c = c

        display_trj = DisplayTrajectory()
        display_trj.trajectory.append(trj)
        display_trj.trajectory_start = state
        print(f"display trajectory: {display_trj}")
        pubs[path].publish(display_trj)
        pubs[path].publish(display_trj)

    rospy.spin()
