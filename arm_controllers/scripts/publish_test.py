#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import Tkinter as tk
from math import floor, ceil

def callback(data):
    global released_1, released_2, released_3, arg
    if data.data[0] == -999.9 and data.data[1] == -999.9:
        released_1 = False
        released_2 = False
        released_3 = False
        arg.data = [0.1, 0.3, 0.2, 0.22, 0.45, 0.0]

pub=rospy.Publisher('/elfin/gravity_comp_controller/command',Float64MultiArray,queue_size = 1)
rospy.init_node('state_listener')
rospy.Subscriber("/elfin/gravity_comp_controller/fail_state", Float64MultiArray, callback)



def normalize(num, lower=-3.14, upper=3.14, b=False):
    res = num
    if not b:
        if lower >= upper:
            raise ValueError("Invalid lower and upper limits: (%s, %s)" %
                             (lower, upper))

        res = num
        if num > upper or num == lower:
            num = lower + abs(num + upper) % (abs(lower) + abs(upper))
        if num < lower or num == upper:
            num = upper - abs(num - lower) % (abs(lower) + abs(upper))

        res = lower if res == upper else num
    else:
        total_length = abs(lower) + abs(upper)
        if num < -total_length:
            num += ceil(num / (-2 * total_length)) * 2 * total_length
        if num > total_length:
            num -= floor(num / (2 * total_length)) * 2 * total_length
        if num > upper:
            num = total_length - num
        if num < lower:
            num = -total_length - num

        res = num * 1.0  # Make all numbers float, to be consistent

    return res



def toggle_1():
    global released_1
    if released_1:
        released_1 = False
    else:
        released_1 = True

def toggle_2():
    global released_2
    if released_2:
        released_2 = False
    else:
        released_2 = True

def toggle_3():
    global released_3
    if released_3:
        released_3 = False
    else:
        released_3 = True


def run():
    global arg, speed1, speed2, speed3, released_1, released_2, released_3
    prev_val1 = arg.data[0]
    prev_val2 = arg.data[1]
    prev_val3 = arg.data[2]
    print(released_1, released_2, released_3)
    if released_1:
        prev_val1 += speed1
    if released_2:
        prev_val2 += speed2
    if released_3:
        prev_val3 += speed3
    arg.data = [prev_val1, prev_val2, prev_val3, 0.22, 0.45, 0.0]
    pub.publish(arg)

    print(arg.data[0], speed1, speed2)
    if arg.data[0] <= -3.14:
        speed1 = 0.02
    elif arg.data[0] >= 3.14:
        speed1 = -0.02
    if arg.data[1] <= -1.0:
        speed2 = 0.01
    elif arg.data[1] >= 0.5:
        speed2 = -0.01
    if arg.data[2] <= 0.0:
        speed3 = 0.01
    elif arg.data[2] >= 1.2:
        speed3 = -0.01
    root.after(50, run)


# --- main ---

root = tk.Tk()
root.geometry("200x110-0+0")
root.title("JOINT CONTROL")

released_1 = False
released_2 = False
released_3 = False
speed1 = 0.03
speed2 = 0.02
speed3 = 0.02

arg = Float64MultiArray()
thing = MultiArrayDimension()
thing.label = ""
thing.size = 6
thing.stride = 0
arg.layout.dim.append(thing)
arg.data = [0.1, 0.3, 0.2, 0.22, 0.45, 0.0]


# ntxt/bck without arguments because you need `global`
tk.Button(root, text="Toggle Joint 1", command=toggle_1).pack()
tk.Button(root, text="Toggle Joint 2", command=toggle_2).pack()
tk.Button(root, text="Toggle Joint 3", command=toggle_3).pack()

run() # without arguments because you need `global`

root.mainloop()

