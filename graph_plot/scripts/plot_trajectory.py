#!/usr/bin/env python3
import rospy
import moveit_msgs.msg
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

import threading

stop_loop : bool = False
loop_animation = None

def loop(display_path,joint_name,speed):
    state = RobotState()
    state.joint_state.name = joint_name
    state.joint_state.velocity = [0] * len(joint_name)
    state.joint_state.effort = [0] * len(joint_name)

    pubs = dict()

    while not rospy.is_shutdown() and not stop_loop:
        for path in display_path:
            pubs[path] = rospy.Publisher(path, DisplayTrajectory, queue_size=10, latch=True)

            try:
                points = rospy.get_param("~"+path)
            except:
                rospy.logerr_throttle(2,f"problems loading path {path}")
                continue

            first_time = True  # true if first point
            trj = RobotTrajectory()
            trj.joint_trajectory.joint_names = joint_name
            trj.joint_trajectory.header.frame_id="world"
            for c in points:
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
            pubs[path].publish(display_trj)
            pubs[path].publish(display_trj)
            rospy.sleep(10)
    
    return 

def plot_trajectory(req: TriggerRequest) -> TriggerResponse:

    try:
        display_path = rospy.get_param("~display_path")
        joint_name = rospy.get_param("~joint_names")
        speed = rospy.get_param("~speed",1)
    except (KeyError, rospy.ROSException) as e:
        res = TriggerResponse()
        res.success = False
        res.message = f'Failed service in getting param {e}'
        return res
        
    global loop_animation

    if loop_animation is None:
        stop_loop = False
        loop_animation = threading.Thread(target=loop, args=(display_path,joint_name,speed))
        loop_animation.start()
    else:
        stop_loop = True
        loop_animation.join()
        loop_animation = threading.Thread(target=loop)
        stop_loop = False

    res = TriggerResponse()
    res.success = True
    res.message = f'Looping the aniumation under the topic(s) \'/{str(display_path)}\' (Type: DisplayTrajectory)'

    return res

if __name__ == "__main__":
    rospy.init_node('plot_trajectory')

    _ = rospy.Service('/start_display_trajectory', Trigger, plot_trajectory)
        
    rospy.spin()
