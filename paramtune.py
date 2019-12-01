#!/usr/bin/env python
# license removed for brevity
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32
import time
import traceback
import Tkinter as tk
import ttk
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import math
import rosparam
import yaml
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg

def B_onclick():
    global eList
    row_para = [float(eList[0].get()), float(eList[1].get())]
    pitch_para = [float(eList[2].get()), float(eList[3].get())]
    turnKp = float(eList[4].get())
    depthKp = float(eList[5].get())
    pub_data = Float32MultiArray(data = row_para)
    pub1.publish(pub_data)
    pub_data = Float32MultiArray(data = pitch_para)
    pub2.publish(pub_data)
    pub3.publish(turnKp)
    pub4.publish(depthKp)
def print_selec_value(data):
    pass
def get_digit(data):
    if data>=1:
        digit =int(math.log(data)/math.log(10))
        num =(data/10**digit)
    elif data==0:
        digit = 0
        num =0
    else:
        digit = int(math.log(data)/math.log(10))-1
        num =data/10**digit
    return digit ,num
class Main():
    def __init__(self):
        rospy.init_node('para_tune',anonymous=True)
        pub1 = rospy.Publisher('/PIDpara/row',Float32MultiArray,queue_size=10)
        pub2 = rospy.Publisher('/PIDpara/pitch',Float32MultiArray,queue_size=10)
        pub3 = rospy.Publisher('/PIDpara/turn',Float32,queue_size=10)

        win = tk.Tk()
        win.title('Dummy motor')
        win.wm_geometry("1500x500")
        scale_frame = tk.Frame(win)
        scale_frame.grid(row = 1 ,column = 1)
        graph = tk.Frame(win)
        graph.grid(row = 1 ,column = 2)
        ################################################################################
        #                               depth PID                                      #
        ################################################################################
        depth_PID=rospy.get_param('/PIDpara/depth')
        depth_PID_digit = []
        depth_PID_num = []
        for i in depth_PID:
            a,b = get_digit(i)
            depth_PID_digit.append(a)
            depth_PID_num.append(b) 
        combox_num = range(-5,5,1)
        tk.Label(scale_frame, text = 'depth_P').grid(row=1, column=0)
        self.s_d_P = tk.Scale(scale_frame,from_=0,to=9.99,orient=tk.HORIZONTAL,length=200,showvalue=1,tickinterval=3,resolution=0.01,command=print_selec_value)
        self.s_d_P.set(depth_PID_num[0])
        self.s_d_P.grid(row=1, column=1)
        self.combo_d_P = ttk.Combobox(scale_frame, values=combox_num,width=5)
        self.combo_d_P.current(depth_PID_digit[0]+5)
        self.combo_d_P.grid(row=1,column=2)
        tk.Label(scale_frame, text = 'depth_I').grid(row=2, column=0)
        self.s_d_I = tk.Scale(scale_frame,from_=0,to=9.99,orient=tk.HORIZONTAL,length=200,showvalue=1,tickinterval=3,resolution=0.01,command=print_selec_value)
        self.s_d_I.set(depth_PID_num[1])
        self.s_d_I.grid(row=2 ,column=1)
        self.combo_d_I = ttk.Combobox(scale_frame, values=combox_num,width=5)
        self.combo_d_I.current(depth_PID_digit[1]+5)
        self.combo_d_I.grid(row=2,column=2)  
        tk.Label(scale_frame, text = 'depth_D').grid(row=3, column=0)
        self.s_d_D = tk.Scale(scale_frame,from_=0,to=9.99,orient=tk.HORIZONTAL,length=200,showvalue=1,tickinterval=3,resolution=0.01,command=print_selec_value)
        self.s_d_D.set(depth_PID_num[2])
        self.s_d_D.grid(row=3 ,column=1)
        self.combo_d_D = ttk.Combobox(scale_frame, values=combox_num,width=5)
        self.combo_d_D.current(depth_PID_digit[2]+5)
        self.combo_d_D.grid(row=3,column=2) 
        tk.Button(scale_frame,  text='set depth param', command=self.set_depth).grid(row = 1 ,column = 5)
        tk.Button(scale_frame,  text='dump depth param', command=self.dump_depth).grid(row = 2 ,column = 5)

        ################################################################################
        #                               altitude PID                                   #
        ################################################################################
        altitude_PID=rospy.get_param('/PIDpara/altitude')
        altitude_PID_digit = []
        altitude_PID_num = []
        for i in altitude_PID:
            a,b = get_digit(i)
            altitude_PID_digit.append(a)
            altitude_PID_num.append(b) 
        combox_num = range(-5,5,1)
        tk.Label(scale_frame, text = 'altitude_P').grid(row=4, column=0)
        self.s_a_P = tk.Scale(scale_frame,from_=0,to=9.99,orient=tk.HORIZONTAL,length=200,showvalue=1,tickinterval=3,resolution=0.01,command=print_selec_value)
        self.s_a_P.set(altitude_PID_num[0])
        self.s_a_P.grid(row=4, column=1)
        self.combo_a_P = ttk.Combobox(scale_frame, values=combox_num,width=5)
        self.combo_a_P.current(altitude_PID_digit[0]+5)
        self.combo_a_P.grid(row=4,column=2)
        tk.Label(scale_frame, text = 'altitude_I').grid(row=5, column=0)
        self.s_a_I = tk.Scale(scale_frame,from_=0,to=9.99,orient=tk.HORIZONTAL,length=200,showvalue=1,tickinterval=3,resolution=0.01,command=print_selec_value)
        self.s_a_I.set(altitude_PID_num[1])
        self.s_a_I.grid(row=5 ,column=1)
        self.combo_a_I = ttk.Combobox(scale_frame, values=combox_num,width=5)
        self.combo_a_I.current(altitude_PID_digit[1]+5)
        self.combo_a_I.grid(row=5,column=2)  
        tk.Label(scale_frame, text = 'altitude_D').grid(row=6, column=0)
        self.s_a_D = tk.Scale(scale_frame,from_=0,to=9.99,orient=tk.HORIZONTAL,length=200,showvalue=1,tickinterval=3,resolution=0.01,command=print_selec_value)
        self.s_a_D.set(altitude_PID_num[2])
        self.s_a_D.grid(row=6 ,column=1)
        self.combo_a_D = ttk.Combobox(scale_frame, values=combox_num,width=5)
        self.combo_a_D.current(altitude_PID_digit[2]+5)
        self.combo_a_D.grid(row=6,column=2) 
        tk.Button(scale_frame,  text='set altitude param', command=self.set_altitude).grid(row = 4 ,column = 5)
        tk.Button(scale_frame,  text='dump altitude param', command=self.dump_altitude).grid(row = 5 ,column = 5)
        ################################################################################
        #                               matplotlib graph                               #
        ################################################################################
        #===============      for depth  ====================#
        self.x_count = 0
        self.depth_x = np.arange(0,100)
        self.ddata =  np.zeros(100)
        self.depth_fig =Figure(figsize=(5,3), dpi=100)
        self.depth_ax=self.depth_fig.add_subplot(111)
        self.depth_ax.set_ylim((0, 5))
        self.depth_ax.set_title("depth")
        self.canvas_d =FigureCanvasTkAgg(self.depth_fig, master=graph)
        self.canvas_d.show()
        self.canvas_d.get_tk_widget().pack(side=tk.TOP,fill="both")
        rospy.Subscriber("/depth", Float32, self.depth_back, queue_size=1)
        scale_frame.mainloop()
    def set_depth(self):
        data = []
        data.append(self.s_d_P.get()*10**float(self.combo_d_P.get()))
        data.append(self.s_d_I.get()*10**float(self.combo_d_I.get()))
        data.append(self.s_d_D.get()*10**float(self.combo_d_D.get()))
        print data
        rosparam.set_param('/PIDpara/depth', str(data))
    def set_altitude(self):
        data = []
        data.append(self.s_a_P.get()*10**float(self.combo_a_P.get()))
        data.append(self.s_a_I.get()*10**float(self.combo_a_I.get()))
        data.append(self.s_a_D.get()*10**float(self.combo_a_D.get()))
        print data
        rosparam.set_param('/PIDpara/altitude', str(data))
    def dump_depth(self):
        data = []
        data.append(self.s_d_P.get()*10**float(self.combo_d_P.get()))
        data.append(self.s_d_I.get()*10**float(self.combo_d_I.get()))
        data.append(self.s_d_D.get()*10**float(self.combo_d_D.get()))
        dict_data = {'/PIDpara/depth':data}
        print dict_data
        with open(r'/home/eason27271563/catkin_ws/src/beginner_tutorials/config/depth.yaml','w+') as f:
            print yaml.dump(dict_data,f, default_flow_style = False)
    def dump_altitude(self):
        data = []
        data.append(self.s_a_P.get()*10**float(self.combo_a_P.get()))
        data.append(self.s_a_I.get()*10**float(self.combo_a_I.get()))
        data.append(self.s_a_D.get()*10**float(self.combo_a_D.get()))
        dict_data = {'/PIDpara/altitude':data}
        print dict_data
        with open(r'/home/eason27271563/catkin_ws/src/beginner_tutorials/config/altitude.yaml','w+') as f:
            print yaml.dump(dict_data,f, default_flow_style = False)
    def depth_back(self,data):
        #print(data.data)
        if self.x_count>99:
            self.x_count+=1
            
            self.ddata=np.roll(self.ddata,-1)
            self.ddata[99]=data.data
            self.depth_x=np.roll(self.depth_x,-1)
            self.depth_x[99]=self.depth_x[98]+1
            self.depth_ax.cla()
            self.depth_ax.set_xlim((self.depth_x[0], self.depth_x[99]))
            self.depth_ax.set_ylim((0, 5))
            self.depth_ax.plot(self.depth_x,self.ddata)
            print(self.depth_x[99])
        else:
            self.ddata[self.x_count]=data.data
            self.x_count+=1
            
            self.depth_ax.cla()
            self.depth_ax.set_ylim((0, 5))
            self.depth_ax.plot(self.depth_x,self.ddata)
        self.depth_ax.set_title("depth")
        self.canvas_d.draw()
if __name__ == "__main__":
    try:
        Main()
    except Exception as e:
        exstr = traceback.format_exc()
        print(exstr)
