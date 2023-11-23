#!/usr/bin/env python3
import rospy
import rosservice
import moveit_msgs.msg
import actionlib
import random
import time
import sys


if __name__ == "__main__":
    rospy.init_node('plot_path')

    display_path=rospy.get_param("~display_path")
    sleep_time=rospy.get_param("~sleep", 0.1)
    jname=rospy.get_param("joint_names")
    ik_sol=moveit_msgs.msg.DisplayRobotState()
    ik_sol.state.joint_state.name=jname
    ik_sol.state.joint_state.velocity= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ik_sol.state.joint_state.effort= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    state_pub = rospy.Publisher('/path',moveit_msgs.msg.DisplayRobotState,queue_size=10)

    #r = rospy.Rate(10) # all solution in 10 seconds
    first_time=True
    last_c=[]

    while (not rospy.is_shutdown()):
        for idx in range(0,len(display_path)):
            try:
                path=rospy.get_param(display_path[idx])
            except:
                print("problems loading path")
                continue

            for c in path:
                if first_time:
                    first_time=False
                    last_c=c
                ik_sol.state.joint_state.position=c
                state_pub.publish(ik_sol)
                distance=max([abs(j-i) for i, j in zip(c, last_c)])
                rospy.sleep(min(sleep_time,0.5*distance))
                last_c=c

                if rospy.is_shutdown():
                    exit()
