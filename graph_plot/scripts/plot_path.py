#!/usr/bin/env python3
import rospy
import moveit_msgs.msg

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

import threading

stop_loop : bool = False
loop_animation = None

def loop(display_path,sleep_time,jname):

    ik_sol=moveit_msgs.msg.DisplayRobotState()
    ik_sol.state.joint_state.name=jname
    ik_sol.state.joint_state.velocity= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ik_sol.state.joint_state.effort= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    state_pub = rospy.Publisher('/path',moveit_msgs.msg.DisplayRobotState,queue_size=10)

    #r = rospy.Rate(10) # all solution in 10 seconds
    first_time=True
    last_c=[]

    while (not rospy.is_shutdown() and not stop_loop):
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

def plot_trajectory(req: TriggerRequest) -> TriggerResponse:
    
    try:
        display_path=rospy.get_param("~display_path")
        sleep_time=rospy.get_param("~sleep", 0.1)
        jname=rospy.get_param("~joint_names")
    except (KeyError, rospy.ROSException) as e:
        res = TriggerResponse()
        res.success = False
        res.message = f'Failed service in getting param {e}'
        return res

    global loop_animation

    if loop_animation is None:
        stop_loop = False
        loop_animation = threading.Thread(target=loop, args=(display_path,sleep_time,jname))
        loop_animation.start()
    else:
        stop_loop = True
        loop_animation.join()
        loop_animation = threading.Thread(target=loop)
        stop_loop = False

    res = TriggerResponse()
    res.success = True
    res.message = f'Looping the aniumation under the topic \'/path\' (Type: DisplayRobotState)'

    return res

if __name__ == "__main__":
    
    rospy.init_node('plot_path')

    _ = rospy.Service('/start_plot_robot_states', Trigger, plot_trajectory)
        
    rospy.spin()
