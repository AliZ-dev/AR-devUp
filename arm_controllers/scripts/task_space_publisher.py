#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import tkinter as tk
from math import floor, ceil, pi


pub=rospy.Publisher('/elfin/task_space_controller/command',Float64MultiArray,queue_size = 1)
rospy.init_node('state_listener')


def toggle_1():
    global x_val, y_val, z_val, z_rot
    x_val = float(0.1)
    y_val = float(-0.3)
    z_val = float(0.56)
    z_rot = pi/4
    print("Position set to", x_val, y_val, z_val)

def toggle_2():
    global x_val, y_val, z_val, z_rot
    x_val = float(0.0)
    y_val = float(-0.32)
    z_val = float(0.56)
    z_rot = 0.0
    print("Position set to", x_val, y_val, z_val)

def toggle_3():
    global x_val, y_val, z_val, z_rot
    x_val = float(0.0)
    y_val = float(0.32)
    z_val = float(0.56)
    z_rot = pi
    print("Position set to", x_val, y_val, z_val)


def run():
    global arg, x_val, y_val, z_val, z_rot

    arg.data = [x_val, y_val, z_val, 0.0, 0.0, z_rot]
    pub.publish(arg)

    root.after(50, run)


# --- main ---

root = tk.Tk()
root.geometry("200x110-0+0")
root.title("POS CONTROL")


arg = Float64MultiArray()
thing = MultiArrayDimension()
thing.label = ""
thing.size = 6
thing.stride = 0
arg.layout.dim.append(thing)
x_val = -0.2
y_val = -0.32
z_val = 0.56
z_rot = 0.0


# ntxt/bck without arguments because you need `global`
tk.Button(root, text="Toggle Pos 1", command=toggle_1).pack()
tk.Button(root, text="Toggle Pos 2", command=toggle_2).pack()
tk.Button(root, text="Toggle Pos 3", command=toggle_3).pack()

run() # without arguments because you need `global`

root.mainloop()

