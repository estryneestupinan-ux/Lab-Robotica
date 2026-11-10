#!/usr/bin/env python3
"""GUI NUEVA de solo lectura: referencias/posiciones de las 4 juntas desde ROS 2.
No envia referencias, no activa actuadores ni sustituye GUI original del usuario.
"""
import math
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState

NAMES = ('joint_1','joint_2','joint_3','joint_4')

class Listener(Node):
    def __init__(self):
        super().__init__('robot2_gui_lectura')
        self.data = None
        self.create_subscription(JointTrajectoryControllerState,
                                 '/arm_controller/controller_state', self.cb, 10)
    def cb(self, msg):
        try:
            indices=[msg.joint_names.index(j) for j in NAMES]
            ref=[math.degrees(msg.reference.positions[i]) for i in indices]
            actual=[math.degrees(msg.feedback.positions[i]) for i in indices] if len(msg.feedback.positions) >= len(msg.joint_names) else None
            self.data=(ref,actual)
        except (ValueError,IndexError,AttributeError):
            return


def main():
    rclpy.init()
    node=Listener()
    root=tk.Tk()
    root.title('Robot2 | Monitor ROS 2 - solo lectura')
    root.geometry('700x340')
    mainframe=ttk.Frame(root,padding=16)
    mainframe.pack(fill='both',expand=True)
    ttk.Label(mainframe,text='Robot2 — referencias y estados en Gazebo',font=('DejaVu Sans',16,'bold')).grid(row=0,column=0,columnspan=4,pady=(0,15))
    for col,text in enumerate(['Articulación','Referencia (°)','Actual Gazebo (°)','Error (°)']):
        ttk.Label(mainframe,text=text,font=('DejaVu Sans',10,'bold')).grid(row=1,column=col,padx=10,pady=8)
    vars_=[]
    for i,name in enumerate(NAMES):
        ttk.Label(mainframe,text=name).grid(row=2+i,column=0,pady=8)
        row=[tk.StringVar(value='—') for _ in range(3)]
        for col,v in enumerate(row,1):
            ttk.Label(mainframe,textvariable=v).grid(row=2+i,column=col,padx=10)
        vars_.append(row)
    state=tk.StringVar(value='Esperando /arm_controller/controller_state ...')
    ttk.Label(mainframe,textvariable=state).grid(row=7,column=0,columnspan=4,pady=(20,0))
    ttk.Label(mainframe,text='Solo monitoreo ROS 2. No envía órdenes a la ESP32.').grid(row=8,column=0,columnspan=4,pady=5)
    def tick():
        if not rclpy.ok():return
        rclpy.spin_once(node,timeout_sec=0)
        if node.data:
            ref,actual=node.data
            for i in range(4):
                vars_[i][0].set('%.2f'%ref[i])
                vars_[i][1].set('%.2f'%actual[i] if actual else '—')
                vars_[i][2].set('%.2f'%(ref[i]-actual[i]) if actual else '—')
            state.set('Recibiendo referencias y feedback')
        root.after(100,tick)
    def close():
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        root.destroy()
    root.protocol('WM_DELETE_WINDOW',close)
    root.after(100,tick)
    root.mainloop()
if __name__ == '__main__':main()
