#!/usr/bin/env python
# -*- coding: utf-8 -*-

from numpy.core.arrayprint import _void_scalar_repr
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64MultiArray, Int64, String
import numpy as np
import time

"""
@author 吉田圭佑
color_tracking_node を受け取りモータの移動量を決定するプログラム

"""


 #Tマーカが横方向に1px動いたときの操作量
WIDE=1

#Tマーカが奥行き方向に1px動いたときの操作量
HIGHT=3

#誤差の許容範囲
limit=0.1

#ロボットの並進方向の決定
def calc_val_velocity(val):
    new_val=val/HIGHT
    if new_val<limit :
        val=val
    else :
        val=new_val
    return val 

#ロボットのヨー角の変更
def calc_val_angle(var):
   #TO DO ここはval=0がいいのかnew_val=valがいいのか考える
    if  abs(var)<limit :
        val=0
        return var
    else :    
        if var<0:
            var=var*WIDE
        else :
            var=-var*WIDE
        
        return var   

class Motor_Run():  
    def __init__(self):
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.twist=Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        #ロボットのロール、ヨー、ピッチ角の初期値を設定
        self.twist.angular.x=0
        self.twist.angular.y=0
        self.twist.angular.z=0
        #x,y,z方向の移動距離（初期値）
        self.movement=0


    def sub_clortracking(self):
        sub1=rospy.Subscriber('robot_position', Int64MultiArray,self.top_position)
        sub2=rospy.Subscriber('yaw_axis_data', Int64,self.bottom_position)
        sub3=rospy.Subscriber('debug',Int64MultiArray,self.depth)
        self.variation=self.top_position-self.bottom_position
        self.twist.linear.x=calc_val_angle(self.variation)
        self.twist.angular.z=calc_val_velocity(sub3)
        self.publisher.Publish(self.twist)
        rospy.spin()



def main():
    while not rospy.is_shutdown():
        temp_time = time.time()
        speed = rospy.get_param("~speed", 0.5)
        turn = rospy.get_param("~turn", 1.0)               
        a=Motor_Run()
        a.sub_clortracking()
        control_cycle = time.time() - temp_time
        rospy.logerr(control_cycle)






if __name__=="__main__":
    rospy.init_node('motor_twist')
    main()
  
        
        