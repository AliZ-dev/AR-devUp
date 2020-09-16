#!/usr/bin/env python
import rospy
from arm_controllers.msg import ControllerJointState
from std_msgs.msg import Float64MultiArray, String, MultiArrayDimension
import math

pub=rospy.Publisher('/elfin/gravity_comp_controller/fail_state', Float64MultiArray, queue_size = 1)
failsafe = False
starting_pos = [0.1, 0.3, 0.2, 0.22, 0.45, 0.0]
fail_arg = Float64MultiArray()
thing = MultiArrayDimension()
thing.label = ""
thing.size = 6
thing.stride = 0
fail_arg.layout.dim.append(thing)
fail_arg.data = starting_pos


def at_start_pos(state):
    idx = 0
    for j in state:
        if abs(math.radians(j) - starting_pos[idx]) > 0.05:
            return False
        idx += 1
    return True


def next_step_to_start_pos(state):
    next_step = []
    j_idx = 0
    for j in state:
        if abs(starting_pos[j_idx] - j) < 0.1:
            next_step.append(starting_pos[j_idx])
        elif starting_pos[j_idx] - j < 0:
            next_step.append(j - 0.01)
        elif starting_pos[j_idx] - j > 0:
            next_step.append(j + 0.01)
        j_idx += 1
    return next_step



def callback(data):
    global failsafe, fail_arg
    #rospy.loginfo(rospy.get_caller_id() + "Values:\n" + "Command: " + str(math.radians(data.command)) \
    #              + "\nCommand dot: " + str(math.radians(data.command_dot)) \
    #              + "\nState: " + str(math.radians(data.state)) \
    #              + "\nState_dot: " + str(math.radians(data.state_dot)))
    joint_num = 1
    for j in data.command:
        print("\njoint", str(joint_num), ":", str(math.radians(j)))
        joint_num += 1
    print("\n")
    j1 = math.radians(data.state[0])
    j2 = math.radians(data.state[1])
    j3 = math.radians(data.state[2])
    j4 = math.radians(data.state[3])
    j5 = math.radians(data.state[4])
    j6 = math.radians(data.state[5])

    if failsafe and at_start_pos(data.state):
        failsafe = False
        fail_arg.data = [-999.9, -999.9, 0, 0, 0, 0]
        pub.publish(fail_arg)

    elif failsafe:
        fail_arg.data = next_step_to_start_pos([j1, j2, j3, j4, j5, j6])
        print("STATE:",[j1, j2, j3, j4, j5, j6])
        print("NEXT STEP:",fail_arg.data)
        pub.publish(fail_arg)

    elif j2 < -0.8 and j3 > 0.5:
        failsafe = True





def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('limiter', anonymous=True)
    rospy.Subscriber("/elfin/gravity_comp_controller/state", ControllerJointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()