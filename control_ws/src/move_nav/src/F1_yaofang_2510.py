#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Empty
import os
import random
from std_msgs.msg import Bool

class MoveBaseSquare():

    def __init__(self):
        rospy.init_node('nav_pharmacy', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.detection_control_pub = rospy.Publisher('/detection_control', Bool, queue_size=10)

        # Create a list to hold the target quaternions (orientations)创建一个列表，保存目标的角度数据
        quaternions = list()
        # 定义四个顶角处机器人的方向角度（Euler angles:http://zh.wikipedia.org/wiki/%E6%AC%A7%E6%8B%89%E8%A7%92)

        euler_angles = (pi/2,pi/2, pi/2,-pi/2, -pi/2, -pi/2,-pi/2,0,-pi,0)
        # Then convert the angles to quaternions，将上面的Euler angles转换成Quaternion的格式

        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

        # 创建一个列表存储导航点的位置
        waypoints = list()
        waypoints.append(Pose(Point(3.45, -2.10, 0), quaternions[0]))    #C
        waypoints.append(Pose(Point(2.75, -1.68, 0), quaternions[1]))    #A 
        waypoints.append(Pose(Point(3.43, -1.22, 0), quaternions[2]))    #B 
        waypoints.append(Pose(Point(1.14, -3.6, 0), quaternions[3]))     #4
        waypoints.append(Pose(Point(0.45, -3.29, 0), quaternions[4]))    #3 
        waypoints.append(Pose(Point(1.14, -2.52, 0), quaternions[5]))    #2
        waypoints.append(Pose(Point(0.42, -2.08, 0), quaternions[6]))    #1
        waypoints.append(Pose(Point(2.49,-4.33,0), quaternions[7]))      #起点
        waypoints.append(Pose(Point(1.31,-0.534,0), quaternions[8]))     #答题区(识别板2) 
        waypoints.append(Pose(Point(3.00,-4.4,0), quaternions[9]))       #识别板1

        self.count = 9  #状态
        self.windows_ABC = 0
        self.windows_A = 1
        self.windows_B = 1
        self.windows_C = 1
        self.windows_1234 = 3
        self.windows_count = 3
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        self.cam_sub = rospy.Subscriber('/cam_return', Int32MultiArray, self.detect_result,queue_size=10)     #订阅检测的药品和窗口数组
        #ram_result=[self.windows_C ,self.windows_A ,self.windows_B ,self.windows_1234 ,self.windows_count]=[是否去c，是否去a，是否去b，样品数量，对应窗口编号]
        self.ram_result = [1, 1, 1, 3, 4]
        rospy.sleep(1)
        
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        rospy.loginfo("Starting navigation...")
        
        # 初始化一个计数器，记录到达的顶点号
        while(not rospy.is_shutdown()):#如果ros系统正常运行
            #有限状态机
            if(self.count == 9):#从起点到识别区
                rospy.loginfo("从起点到识别区")
                goal = self.create_goal(waypoints[9])
                self.detection_control_pub.publish(Bool(True))
                rospy.loginfo("QR Code detection enabled")
                if(self.move(goal) == True):
                    rospy.loginfo("到达识别区。。。。。。。。")
                    self.count = 10
                    rospy.sleep(5)    #给10秒钟识别时间 10
                    self.detection_control_pub.publish(Bool(True))
                    rospy.loginfo("QR Code detection enabled")
                    
            elif(self.count == 10):#从起点到配药区
                self.detection_control_pub.publish(Bool(False))
                rospy.loginfo("QR Code detection disabled")
                rospy.loginfo('this count 10, start to go to pharmacy area')
                success = False
                rospy.loginfo("self.windows_A="+str(self.windows_A))
                rospy.loginfo("self.windows_B="+str(self.windows_B))
                rospy.loginfo("self.windows_C="+str(self.windows_C))
                rospy.loginfo("self.windows_1234="+str(self.windows_1234))
                if(self.windows_C == 1):
                    goal = self.create_goal(waypoints[0])
               
                    if(self.move(goal) == True):
                        rospy.loginfo("取到c窗口中的")
                        success = True
                        self.clear_costmaps_service()
                        rospy.sleep(1)      
                if(self.windows_A == 1 ):
                    goal = self.create_goal(waypoints[1])
                    if(self.move(goal) == True):
                        rospy.loginfo("取到a窗口中的")
                        success = True
                        self.clear_costmaps_service()
                        rospy.sleep(1)    
                if(self.windows_B == 1 ):
                    goal = self.create_goal(waypoints[2])
                    if(self.move(goal) == True):
                        rospy.loginfo("取到b窗口中的")
                        success = True
                        self.clear_costmaps_service()
                        rospy.sleep(1) 
                if success:
                    self.count = 11
                else:
                    rospy.logwarn("未能成功到达任何配药窗口")
                    continue

                #播报样本类型
                if(self.windows_1234 == 3): #4(激素检验窗口)
                    rospy.loginfo("血浆样本") #后面加播报
                elif(self.windows_1234 == 4): #3(免疫检测窗口)
                    rospy.loginfo("组织样本") #后面加播报
                elif(self.windows_1234 == 5): #2(体液窗口)
                    rospy.loginfo("唾液样本") #后面加播报
                elif(self.windows_1234 == 6): #1(血常规窗口)
                    rospy.loginfo("静脉血样本") #后面加播报

            elif(self.count == 11):#从配药区到答题区
                rospy.loginfo("从配药区到答题区路上")
                goal = self.create_goal(waypoints[8])
                if(self.move(goal) == True):
                    rospy.loginfo("到达识别板2。。。。。。。。")
                    self.count = 12
                    self.clear_costmaps_service()   #清除产生的偏移代价地图
                    rospy.loginfo("化验区无空闲，等待中")
                    rospy.sleep(1)     #在此之后加语音播报

            elif(self.count == 12):#从答题区到数字区
                rospy.loginfo("从答题区到数字区路上")
                goal = self.create_goal(waypoints[(6 - self.windows_1234)])
                if(self.move(goal) == True):
                    rospy.loginfo("到达数字区。。。。。。。。")                    
                    self.count = 9
                    self.clear_costmaps_service()
                    rospy.sleep(1)     
                    temp = '/home/EPRobot/robot_ws/src/pharmacy_pkg/'
                    A4 = '/yuyingwenjian/4.wav'
                    A3 = '/yuyingwenjian/3.wav'
                    A2 = '/yuyingwenjian/2.wav'
                    A1 = '/yuyingwenjian/1.wav'
                    try:
                        if self.windows_1234 == 3:
                            os.system('play '+temp+A4)
                            rospy.loginfo("到达4号激素检验窗口")
                        elif self.windows_1234 == 4:
                            os.system("play "+temp+A3)
                            rospy.loginfo("到达3号免疫检验窗口")
                        elif self.windows_1234 == 5:
                            os.system("play "+temp+A2)
                            rospy.loginfo("到达2号体液检验窗口")
                        elif self.windows_1234 == 6:
                            os.system("play "+temp+A1)
                            rospy.loginfo("到达1号血常规检验窗口")
                    except Exception as e:
                        rospy.logerr("播放音频失败: %s", str(e))
                    #播报样本数量
                    if self.windows_count == 3:
                            rospy.loginfo("样本数为3")
                    elif self.windows_count == 2:
                            rospy.loginfo("样本数为2")
                    elif self.windows_count == 1:
                            rospy.loginfo("样本数为1")

    def create_goal(self, pose):
        goal = MoveBaseGoal()  
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        return goal

    def move(self, goal):
        self.move_base.send_goal(goal)
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                return True
        return False        

    def detect_result(self, msg):
        if self.count == 10:
            self.ram_result = msg.data
            rospy.logwarn("self.ram_result: %s", self.ram_result)
            self.windows_C = self.ram_result[0]
            self.windows_A = self.ram_result[1]
            self.windows_B = self.ram_result[2]
            self.windows_count = self.ram_result[3]
            self.windows_1234 = self.ram_result[4]

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

  
if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")