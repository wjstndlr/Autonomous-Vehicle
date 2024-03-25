#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from sensor_msgs.msg import Image, CompressedImage, Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from math import *
import time
import os
from cv_bridge import CvBridge
import cv2
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers


#=============================================
# 차선 인식 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30  # 카메라 FPS 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
ROI_START_ROW = 400  # 차선을 찾을 ROI 영역의 시작 Row값
ROI_END_ROW = 480  # 차선을 찾을 ROT 영역의 끝 Row값
ROI_HEIGHT = ROI_END_ROW - ROI_START_ROW  # ROI 영역의 세로 크기  
L_ROW = 40  # 차선의 위치를 찾기 위한 ROI 안에서의 기준 Row값 
View_Center = 320 #WIDTH//2
Blue =  (255,0,0) # 파란색
Green = (0,255,0) # 녹색
Red =   (0,0,255) # 빨간색
Yellow = (0,255,255) # 노란색



#=============================================
#정지선 프로그램에서 사용할 변수 선언부
#=============================================
White_dot_count = 150


class Subsrcibe_test:
   
    def __init__ (self):
        rospy.init_node("wego_sub_node")
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.compressed_callback)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber('/imu', Imu, callback=self.imu_CB)
        rospy.Subscriber('/path_', String, self.V2X_callback)
        rospy.Subscriber('/camera/ar_pose_marker',  AlvarMarkers, self.ar_callback, queue_size=1 )
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cvbridge = CvBridge()
        self.twist = Twist()
        self.image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.lineweight = 495
        self.start_time = time.time()
        self.ranges = []
        self.degrees = []
        self.avoid_check_count = 20
        self.fixed_speed = 0.2
        self.fixed_angle = 0.0
        self.imu = 0.0
        self.imu_degree = 0.0
        self.pre_degree = 0.0
        self.weight= 0.01
        self.data = ""
        self.roll_x = 0.0
        self.pitch_y = 0.0
        self.yaw_z = 0.0
        self.ar_msg =  {"ID":[],"DX":[],"DZ":[]}  # AR태그 토픽을 담을 변수



    def drive(self,angle,speed):
        self.twist.linear.x = speed
        self.twist.angular.z = angle   
        self.vel_pub.publish(self.twist)
    
    def V2X_callback(self, msg):
        self.data = msg.data

    
    def stop(self,sleep_sec):
        for i in range(sleep_sec*1): 
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.vel_pub.publish(self.twist)
            time.sleep(0.2)
   
    def compressed_callback(self, msg):
       self.image=self.cvbridge.compressed_imgmsg_to_cv2(msg)
    #    cv2.imshow("Camera Image", self.image)
    #    cv2.waitKey(1)
    
    def lidar_callback(self,msg):
       self.degrees = [(msg.angle_min + msg.angle_increment*index)* 180/pi for index, value in enumerate(msg.ranges)]
       self.ranges = msg.ranges 
  
    def ar_callback(self,data):
        # AR태그의 ID값, X 위치값, Z 위치값을 담을 빈 리스트 준비
        self.ar_msg["ID"] = []
        self.ar_msg["DX"] = []
        self.ar_msg["DY"] = []

        # 발견된 모두 AR태그에 대해서 정보 수집하여 ar_msg 리스트에 담음
        for i in data.markers:
            self.ar_msg["ID"].append(i.id) # AR태그의 ID값을 리스트에 추가
            self.ar_msg["DX"].append(i.pose.pose.position.x) # AR태그의 X 위치값을 리스트에 추가
            self.ar_msg["DY"].append(i.pose.pose.position.y) # AR태그의 Z 위치값을 리스트에 추가   
        # self.ar_msg["ID"]=data.markers.id # AR태그의 ID값을 리스트에 추가
        # self.ar_msg["DX"]=data.pose.pose.position.x # AR태그의 X 위치값을 리스트에 추가
        # self.ar_msg["DY"]=data.pose.pose.position.y # AR태그의 Z 위치값을 리스트에 추가   
    
    def imu_CB(self, msg):       
    #os.system('clear')
        
        self.w = msg.orientation.w
        self.x = msg.orientation.x
        self.y = msg.orientation.y
        self.z = msg.orientation.z
       
        self.roll_x, self.pitch_y, self.yaw_z = self.euler_from_quaternion(self.x, self.y, self.z, self.w)
     
    def object_avoidance(self,degrees,ranges): ## mission3
      
        self.e_stop_check = 0
        self.avoid_check = 0  
        
            
        for index, value in enumerate(self.ranges):
            
            if 0 < abs(degrees[index]) < 20 and 0 < self.ranges[index] < 0.05:
                self.e_stop_check += 1
            
            elif 20 < abs(degrees[index]) <60 and 0 < self.ranges[index] < 0.8:
                self.avoid_check += 1
                  
            else :
                pass
            
            r_list = self.ranges[0:63] #[int(270 * increment)] >  # -90 ~ -53도에 해당하는 라이다 센서의 거리값 변환 (오른쪽1)
            r_list = list(r_list)
            r_list[:] = [value for value in r_list if value != 0] ## 라이다 거리값중 0이 뜨는 노이즈 값은 제거하고 측정되는 값만 필터링하는 과정이다.
            
            fr_list = self.ranges[63:123] #[int(315 * increment)] > 라이다 -53~ -17도에 해당하는 라이다 센서의 거리값 변환 (오른쪽2)
            fr_list = list(fr_list)
            fr_list[:] = [value for value in fr_list if value != 0]
            
            fm_list = self.ranges[123:248] #[int(0 * increment)] > 라이다 -17~17도에 해당하는 라이다 센서의 거리값 변환(전방) (전방)
            fm_list = list(fm_list)
            fm_list[:] = [value for value in fm_list if value != 0]
            
            fl_list = self.ranges[248:307] #[int(45 * increment)] > 라이다 17~53도에 해당하는 라이다 센서의 거리값 변환 (왼쪽1)
            fl_list = list(fl_list)
            fl_list[:] = [value for value in fl_list if value != 0] 
            
            l_list = self.ranges[307:370] #[int(90 * increment)] > 라이다 53~ 90도에 해당하는 라이다 센서의 거리값 변환 (왼쪽2)
            l_list = list(l_list)
            l_list[:] = [value for value in l_list if value != 0]
                    
            r = min(r_list) if len(r_list) != 0 else 0 ## r_list의 길이가 0이 아니면 최솟값이 출력되고, 아닐때는 0이다.
            fr = min(fr_list) if len(fr_list) != 0 else 0
            fm = min(fm_list) if len(fm_list) != 0 else 0
            fl = min(fl_list) if len(fl_list) != 0 else 0
            l = min(l_list) if len(l_list) != 0 else 0
      
            #print("r: "+ str(r) + " fr: " + str(fr) + " fm: " + str(fm) + " fl: " + str(fl) + " l: " + str(l))
                # 앞쪽 가까이에 장애물이 있으면 차량 멈춤
        
         ###################################################### 인지 부분 강화확인 부분 (제어)    
        if self.e_stop_check>50 :
            print("stop")
            self.twist.linear.x = 0
            self.pub.publish(self.twist)
        
        elif self.avoid_check > 20 :
            print("avoid")
            
            right_space = fr
            left_space = fl
            difference_distance = right_space-left_space
            
            if right_space > left_space:
                print("avoid right")
                self.twist.angular.z = -0.75
           
            elif right_space < left_space:
                print("avoid left")
                self.twist.angular.z = 0.65    


            else:
                print("go straight")    
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0.0
        
        else :
            print("safe")
            self.twist.linear.x = self.fixed_speed
            self.twist.angular.z = self.imu
        
        self.vel_pub.publish(self.twist)
        print(self.twist)
        #print("avoid_check:", self.avoid_check)
   
    def object_avoidance1(self,degrees,ranges):   ## mission4 
      
        self.e_stop_check = 0
        self.avoid_check = 0  
        
            
        for index, value in enumerate(self.ranges):
            
            if 0 < abs(degrees[index]) < 20 and 0 < self.ranges[index] < 0.5:
                self.e_stop_check += 1
            
            elif 20 < abs(degrees[index]) <60 and 0 < self.ranges[index] < 0.8:
                self.avoid_check += 1
                  
            else :
                pass
            
            r_list = self.ranges[0:83] #[int(270 * increment)] >  # -90 ~ -53도에 해당하는 라이다 센서의 거리값 변환 (오른쪽1)
            r_list = list(r_list)
            r_list[:] = [value for value in r_list if value != 0] ## 라이다 거리값중 0이 뜨는 노이즈 값은 제거하고 측정되는 값만 필터링하는 과정이다.
            
            fr_list = self.ranges[83:153] #[int(315 * increment)] > 라이다 -53~ -17도에 해당하는 라이다 센서의 거리값 변환 (오른쪽2)
            fr_list = list(fr_list)
            fr_list[:] = [value for value in fr_list if value != 0]
            
            fm_list = self.ranges[153:217] #[int(0 * increment)] > 라이다 -17~17도에 해당하는 라이다 센서의 거리값 변환(전방) (전방)
            fm_list = list(fm_list)
            fm_list[:] = [value for value in fm_list if value != 0]
            
            fl_list = self.ranges[217:287] #[int(45 * increment)] > 라이다 17~53도에 해당하는 라이다 센서의 거리값 변환 (왼쪽1)
            fl_list = list(fl_list)
            fl_list[:] = [value for value in fl_list if value != 0] 
            
            l_list = self.ranges[307:370] #[int(90 * increment)] > 라이다 53~ 90도에 해당하는 라이다 센서의 거리값 변환 (왼쪽2)
            l_list = list(l_list)
            l_list[:] = [value for value in l_list if value != 0]
                    
            r = min(r_list) if len(r_list) != 0 else 0 ## r_list의 길이가 0이 아니면 최솟값이 출력되고, 아닐때는 0이다.
            fr = min(fr_list) if len(fr_list) != 0 else 0
            fm = min(fm_list) if len(fm_list) != 0 else 0
            fl = min(fl_list) if len(fl_list) != 0 else 0
            l = min(l_list) if len(l_list) != 0 else 0
      
            #print("r: "+ str(r) + " fr: " + str(fr) + " fm: " + str(fm) + " fl: " + str(fl) + " l: " + str(l))
                # 앞쪽 가까이에 장애물이 있으면 차량 멈춤
        1.25
         ###################################################### 인지 부분 강화확인 부분 (제어)    
        
        if self.avoid_check > self.avoid_check_count :
            print("avoid")
            print("avoid_check:",self.avoid_check)
            
            self.twist.linear.x = self.fixed_speed
            right_space = fr 
            left_space = fl 
            difference_distance = right_space - left_space
            if right_space > left_space:
                print("avoid right")
                steer_angle = -difference_distance
                # if steer_angle > 0.1:
                #     steer_angle = steer_angle
                # else:
                #     steer_angle *= 10
                # print("right_angle:",steer_angle)
                self.twist.angular.z = steer_angle*1.55
           
            elif right_space < left_space:
                print("avoid left")
                steer_angle = -difference_distance
                # if steer_angle >= 0.1:
                #     steer_angle = steer_angle
                # else:
                #     steer_angle *= 10
                # print("left_angle:",steer_angle)
                self.twist.angular.z = steer_angle *1.3
            


            else:
                print("go straight")    
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0.0
      
        else :
            print("safe")
            self.twist.linear.x = self.fixed_speed
            self.twist.angular.z = self.fixed_angle
        
        self.vel_pub.publish(self.twist)
        print(self.twist)
        print("avoid_check:", self.avoid_check)
   

    def imu_control(self,pre_degree): ## all
        self.imu_degree = (self.yaw_z *180/pi)

        if pre_degree < self.imu_degree:
           
           steer = (pre_degree - self.imu_degree) * self.weight
        
        else: 
           steer = (pre_degree -self.imu_degree) * self.weight
        
        return steer


    def euler_from_quaternion(self, x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in rad
        
    def line_detection(self,image):
    

        prev_x_left = 0
        prev_x_right = WIDTH
        
        # img = image.copy() # 이미지처리를 위한 카메라 원본이미지 저장
        # display_img = img  # 디버깅을 위한 디스플레이용 이미지 저장
        
        # img(원본이미지)의 특정영역(ROI Area)을 잘라내기
        roi_img = image[ROI_START_ROW:ROI_END_ROW, 0:WIDTH]
        
        # cv2.imshow("roi_img",roi_img)
        #line_draw_img = roi_img.copy()

        #=========================================
        # 원본 칼라이미지를 그레이 회색톤 이미지로 변환하고 
        # 블러링 처리를 통해 노이즈를 제거한 후에 (약간 뿌옇게, 부드럽게)
        # Canny 변환을 통해 외곽선 이미지로 만들기
        #=========================================
        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray,(5, 5), 0)
        edge_img = cv2.Canny(np.uint8(blur_gray), 60, 75)
        # threshold1: 이 값 이하에 포함된 그래디언트의 에지는 모두 버려집니다. 즉, 최소 강도 경계입니다.
        # threshold2: 이 값 이상에 포함된 그래디언트의 에지는 에지로 간주됩니다. 이 값은 최대 강도 경계입니다.
        # 두 임계값 수정해줘야 할수 있다.
        
        all_lines = cv2.HoughLinesP(edge_img, 1,pi/180,30,50,20)
        
        # 마지막 3개의 파라미터 필요조정
        # threshold: 직선으로 간주되기 위해 필요한 최소한의 투표(vote) 수. 이 값이 크면 더 긴 선분이 검출되고, 작으면 더 짧은 선분도 검출됩니다.
        # minLineLength: 검출된 선분의 최소 길이. 이 값보다 짧은 선분은 무시됩니다.
        # maxLineGap: 한 선분으로 간주되기 위한 선분 간의 최대 허용 간격. 이 값이 작으면 선분 사이가 가까워야 하나의 선으로 간주되고, 값이 크면 멀리 떨어진 선분들도 하나의 선으로 간주됩니다.

        if all_lines is None:
            return False, 0, 0

        #=========================================
        # 선분들의 기울기 값을 각각 모두 구한 후에 리스트에 담음. 
        # 기울기의 절대값이 너무 작은 경우 (수평선에 가까운 경우)
        # 해당 선분을 빼고 담음. 
        #=========================================
        slopes = []
        filtered_lines = []

        for line in all_lines:
            x1, y1, x2, y2 = line[0]

            if (x2 == x1):
                slope = 1000.0
            else:
                slope = float(y2-y1) / float(x2-x1)
        
            if 0.2 < abs(slope):
                slopes.append(slope) ## slopes는 조건을 만족하는 기울기 값 저장
                filtered_lines.append(line[0]) ## filtered_lines는 조건을 만족하는 리스트 저장

        if len(filtered_lines) == 0:
            return False, 0, 0

        #=========================================
        # 왼쪽 차선에 해당하는 선분과 오른쪽 차선에 해당하는 선분을 구분하여 
        # 각각 별도의 리스트에 담음.
        #=========================================
        left_lines = []
        right_lines = []

        for j in range(len(slopes)):
            Line = filtered_lines[j]
            slope = slopes[j]

            x1,y1, x2,y2 = Line
            # x2,y2가 직선의 윗점( 왼쪽 차선인 경우)
            # x1,y1가 직선의 윗점 ( 오른쪽 차선인 경우 )

            # 기울기 값이 음수이고 화면의 왼쪽에 있으면 왼쪽 차선으로 분류함
            # 기준이 되는 X좌표값 = (화면중심값 - Margin값)
            Margin = 0
            # 조정

            if (slope < 0) and (x2 < WIDTH/2-Margin):
                left_lines.append(Line.tolist())

            # 기울기 값이 양수이고 화면의 오른쪽에 있으면 오른쪽 차선으로 분류함
            # 기준이 되는 X좌표값 = (화면중심값 + Margin값)
            elif (slope > 0) and (x1 > WIDTH/2+Margin):
                right_lines.append(Line.tolist())

        # 디버깅을 위해 차선과 관련된 직선과 선분을 그리기 위한 도화지 준비
        #line_draw_img = roi_img.copy()
        
        # 왼쪽 차선에 해당하는 선분은 빨간색으로 표시
        for line in left_lines:
            if x1 > 60:
                x1,y1, x2,y2 = line
                cv2.line(roi_img, (x1,y1), (x2,y2), Red, 2)


        # 오른쪽 차선에 해당하는 선분은 노란색으로 표시
        for line in right_lines:
            if x1 < 450:
                x1,y1, x2,y2 = line
                cv2.line(roi_img, (x1,y1), (x2,y2), Yellow, 2)

        #=========================================
        # 왼쪽/오른쪽 차선에 해당하는 선분들의 데이터를 적절히 처리해서 
        # 왼쪽차선의 대표직선과 오른쪽차선의 대표직선을 각각 구함.
        # 기울기와 Y절편값으로 표현되는 아래와 같은 직선의 방적식을 사용함.
        # (직선의 방정식) y = mx + b (m은 기울기, b는 Y절편)
        #=========================================

        # 왼쪽 차선을 표시하는 대표직선을 구함        
        m_left, b_left = 0.0, 0.0
        x_sum, y_sum, m_sum = 0.0, 0.0, 0.0

        # 왼쪽 차선을 표시하는 선분들의 기울기와 양끝점들의 평균값을 찾아 대표직선을 구함
        size = len(left_lines)
        if size != 0:
            for line in left_lines:
                x1, y1, x2, y2 = line
                x_sum += x1 + x2
                y_sum += y1 + y2
                if(x2 != x1):
                    m_sum += float(y2-y1)/float(x2-x1)
                else:
                    m_sum += 0                
                
            x_avg = x_sum / (size*2)
            y_avg = y_sum / (size*2)
            m_left = m_sum / size
            b_left = y_avg - m_left * x_avg

            if m_left != 0.0:
                #=========================================
                # (직선 #1) y = mx + b 
                # (직선 #2) y = 0
                # 위 두 직선의 교점의 좌표값 (x1, 0)을 구함.           
                x1 = int((0.0 - b_left) / m_left)

                #=========================================
                # (직선 #1) y = mx + b 
                # (직선 #2) y = ROI_HEIGHT
                # 위 두 직선의 교점의 좌표값 (x2, ROI_HEIGHT)을 구함.               
                x2 = int((ROI_HEIGHT - b_left) / m_left)

                # 두 교점, (x1,0)과 (x2, ROI_HEIGHT)를 잇는 선을 그림
                cv2.line(roi_img, (x1,0), (x2,ROI_HEIGHT), Blue, 2)

        # 오른쪽 차선을 표시하는 대표직선을 구함      
        m_right, b_right = 0.0, 0.0
        x_sum, y_sum, m_sum = 0.0, 0.0, 0.0

        # 오른쪽 차선을 표시하는 선분들의 기울기와 양끝점들의 평균값을 찾아 대표직선을 구함
        size = len(right_lines)
        if size != 0:
            for line in right_lines:
                x1, y1, x2, y2 = line
                x_sum += x1 + x2
                y_sum += y1 + y2
                if(x2 != x1):
                    m_sum += float(y2-y1)/float(x2-x1)
                else:
                    m_sum += 0     
        
            x_avg = x_sum / (size*2)
            y_avg = y_sum / (size*2)
            m_right = m_sum / size
            b_right = y_avg - m_right * x_avg

            if m_right != 0.0:
                #=========================================
                # (직선 #1) y = mx + b 
                # (직선 #2) y = 0
                # 위 두 직선의 교점의 좌표값 (x1, 0)을 구함.           
                x1 = int((0.0 - b_right) / m_right)

                #=========================================
                # (직선 #1) y = mx + b 
                # (직선 #2) y = ROI_HEIGHT
                # 위 두 직선의 교점의 좌표값 (x2, ROI_HEIGHT)을 구함.               
                x2 = int((ROI_HEIGHT - b_right) / m_right)

                # 두 교점, (x1,0)과 (x2, ROI_HEIGHT)를 잇는 선을 그림
                cv2.line(roi_img, (x1,0), (x2,ROI_HEIGHT), Blue, 2)

        #=========================================
        # 차선의 위치를 찾기 위한 기준선(수평선)은 아래와 같음.
        #   (직선의 방정식) y = L_ROW 
        # 위에서 구한 2개의 대표직선, 
        #   (직선의 방정식) y = (m_left)x + (b_left)
        #   (직선의 방정식) y = (m_right)x + (b_right)
        # 기준선(수평선)과 대표직선과의 교점인 x_left와 x_right를 찾음.
        #=========================================

        #=========================================        
        # 대표직선의 기울기 값이 0.0이라는 것은 직선을 찾지 못했다는 의미임
        # 이 경우에는 교점 좌표값을 기존 저장해 놨던 값으로 세팅함 
        #=========================================
        if m_left == 0.0:
            x_left = prev_x_left  # 변수에 저장해 놓았던 이전 값을 가져옴
            
        #=========================================
        # 아래 2개 직선의 교점을 구함
        # (직선의 방정식) y = L_ROW  
        # (직선의 방정식) y = (m_left)x + (b_left)
        #=========================================
        else:
            x_left = int((L_ROW - b_left) / m_left)
                            
        #=========================================
        # 대표직선의 기울기 값이 0.0이라는 것은 직선을 찾지 못했다는 의미임
        # 이 경우에는 교점 좌표값을 기존 저장해 놨던 값으로 세팅함 
        #=========================================
        if m_right == 0.0:
            x_right = prev_x_right  # 변수에 저장해 놓았던 이전 값을 가져옴   
        
        #=========================================
        # 아래 2개 직선의 교점을 구함
        # (직선의 방정식) y = L_ROW  
        # (직선의 방정식) y = (m_right)x + (b_right)
        #=========================================
        else:
            x_right = int((L_ROW - b_right) / m_right)
        
        #=========================================
        # 대표직선의 기울기 값이 0.0이라는 것은 직선을 찾지 못했다는 의미임
        # 이 경우에 반대쪽 차선의 위치 정보를 이용해서 내 위치값을 정함 
        #=========================================
        if m_left == 0.0 and m_right != 0.0:
            x_left = x_right - self.lineweight
            
        ## 왼쪽차선에서는 차선이 검출되지 않고 오른쪽만 검출되는 경우
        ## 조정(380) > 중요
        
        if m_left != 0.0 and m_right == 0.0:
            x_right = x_left + 530
        
        ## 왼쪽에서는 차선이 검출되고 오른쪽에서는 차선이 검출되지 않는 경우
        ## 조정(380) > 중요

        # 이번에 구한 값으로 예전 값을 업데이트 함         
        prev_x_left = x_left
        prev_x_right = x_right
        
        # 왼쪽 차선의 위치와 오른쪽 차선의 위치의 중간 위치를 구함
        x_midpoint = (x_left + x_right) // 2 
        # differnce = x_midpoint- View_Center
        # angle = differnce * 0.005
        # speed = 0.42
        # os.system("clear")
        # print("angle:",angle)
        # print(twist)
        #=========================================
        # 디버깅용 이미지 그리기
        # (1) 수평선 그리기 (직선의 방정식) y = L_ROW 
        # (2) 수평선과 왼쪽 대표직선과의 교점 위치에 작은 녹색 사각형 그리기 
        # (3) 수평선과 오른쪽 대표직선과의 교점 위치에 작은 녹색 사각형 그리기 
        # (4) 왼쪽 교점과 오른쪽 교점의 중점 위치에 작은 파란색 사각형 그리기
        # (5) 화면의 중앙점 위치에 작은 빨간색 사각형 그리기 
        #=========================================
        cv2.line(roi_img, (0,L_ROW), (WIDTH,L_ROW), Yellow, 2)
        cv2.rectangle(roi_img, (x_left-5,L_ROW-5), (x_left+5,L_ROW+5), Green, 4)
        cv2.rectangle(roi_img, (x_right-5,L_ROW-5), (x_right+5,L_ROW+5), Green, 4)
        cv2.rectangle(roi_img, (x_midpoint-5,L_ROW-5), (x_midpoint+5,L_ROW+5), Blue, 4)
        cv2.rectangle(roi_img, (View_Center-5,L_ROW-5), (View_Center+5,L_ROW+5), Red, 4)

        # 위 이미지를 디버깅용 display_img에 overwrite해서 화면에 디스플레이 함
        #display_img[ROI_START_ROW:ROI_END_ROW, 0:WIDTH] = line_draw_img
        cv2.imshow("Lanes_positions", roi_img)
        cv2.waitKey(1)
        #self.drive(-angle,speed)
        print("x_left:",x_left)
        print("x_right:",x_right)
        print("midpoint:",x_midpoint)
    # print(twist)
        ## x_left, x_right이 roi의 중심선과의 교점이라고 할수 있다.
        return True, x_left, x_right
    
    def check_AR(self, target_id=0, stop_distance=0.2):
       
       # 발견된 AR태그 모두에 대해서 조사
            if self.ar_msg["ID"]== target_id:
                self.twist.angular.z = 0.3
                self.vel_pub.publish(self.twist)

                if self.ar_msg["DX"] < stop_distance:
                    self.stop(2)
            else:
                self.twist.angular.z = 0.0
                self.vel_pub.publish(self.twist)
        
def main():
   
    LANE_DRIVE1 = 1
    LIDAR_DRIVE = 2
    LABACON_DRIVE = 3
    LANE_DRIVE2 = 4
    V2X_DRIVE =5
    AR_DRIVE = 6
    subscribe_test = Subsrcibe_test()
    subscribe_test.start_time = time.time()
    print("start_time:",subscribe_test.start_time)
    drive_mode = AR_DRIVE
    
    while not rospy.is_shutdown():
        # subscribe_test.start_time = current_time
        
        if drive_mode ==1 :
            
            found, x_left, x_right = subscribe_test.line_detection(subscribe_test.image)
            current_time = time.time()
            angle = 0.0
            speed = 0.0
            
            if found:
                x_midpoint = (x_left + x_right) // 2
                differnce = x_midpoint- View_Center
                angle = differnce * 0.005
                speed = 0.42
                subscribe_test.drive(-angle,speed)
                time_distance = current_time -  subscribe_test.start_time
                print("current_time:",current_time)
                print("time_distance:", time_distance)
           
            else:
                pass
                
            if current_time - subscribe_test.start_time > 15.3:
                drive_mode = LIDAR_DRIVE
                #  필요한 경우 타이머를 재설정
                subscribe_test.start_time = current_time
                subscribe_test.stop(10)
                print("drive_mode:",drive_mode)
                cv2.destroyWindow("Lanes_positions")
        
        if drive_mode ==2:

            current_time = time.time()
            subscribe_test.pre_degree = -14.2
            obstacle_angle = subscribe_test.imu_control(subscribe_test.pre_degree)
            subscribe_test.imu = obstacle_angle
            time_distance=current_time - subscribe_test.start_time
            print("current_time:",current_time)
            print("time_distance:", time_distance)
            subscribe_test.object_avoidance(subscribe_test.degrees, subscribe_test.ranges)
                
            if current_time - subscribe_test.start_time > 16.2:
                drive_mode = LABACON_DRIVE
                #  필요한 경우 타이머를 재설정
                subscribe_test.start_time = current_time
                subscribe_test.stop(10)
                print("drive_mode:",drive_mode)

        if drive_mode ==3:
            subscribe_test.object_avoidance1(subscribe_test.degrees, subscribe_test.ranges)
            current_time = time.time()
            time_distance=current_time - subscribe_test.start_time
            print("current_time:",current_time)
            print("time_distance:", time_distance)
            if current_time - subscribe_test.start_time > 19.2:
                drive_mode =LANE_DRIVE2
                subscribe_test.start_time = current_time
                subscribe_test.stop(10)

        if drive_mode == 4:
            found, x_left, x_right = subscribe_test.line_detection(subscribe_test.image)
            current_time = time.time()
            angle = 0.0
            speed = 0.0
            if found:
                x_midpoint = (x_left + x_right) // 2
                differnce = x_midpoint- View_Center
                angle = differnce * 0.005
                speed = 0.42
                subscribe_test.drive(-angle,speed)
            else :
                pass
            if current_time - subscribe_test.start_time >8.55 :
                drive_mode = AR_DRIVE
                subscribe_test.start_time = current_time
                subscribe_test.stop(10)    
           
           
        if drive_mode ==5:
            current_time = time.time()
            mode1 = "A"  
            mode2 = "B"
            mode3 = "C"
            # angle = 0.0
            # speed = 0.0

            if subscribe_test.data == mode1:
                if current_time - subscribe_test.start_time > 2.7 :
                    print("b")
                    subscribe_test.lineweight = 250
                    found, x_left, x_right = subscribe_test.line_detection(subscribe_test.image)
                    if found:
                        x_midpoint = (x_left + x_right) // 2
                        differnce = x_midpoint- View_Center
                        angle = differnce * 0.005
                        speed = 0.2
                        subscribe_test.drive(-angle,speed)
                    else :
                        x_midpoint = (x_left + x_right) // 2
                        differnce = x_midpoint- View_Center
                        angle = differnce * 0.005
                        speed = 0.2
                        subscribe_test.drive(-angle,speed)
                    
                else:
                  
                    for i in range(4):
                        subscribe_test.twist.linear.x = 0.2
                        subscribe_test.twist.angular.z = 0.475
                        subscribe_test.vel_pub.publish(subscribe_test.twist)
                        print("a")
            
            if subscribe_test.data == mode2:
               
               if current_time - subscribe_test.start_time > 4.5 :
                    subscribe_test.fixed_angle = -0.15
                    subscribe_test.avoid_check_count = 10
                    subscribe_test.object_avoidance1(subscribe_test.degrees, subscribe_test.ranges)
               else:
                    for i in range(4):
                        subscribe_test.twist.linear.x = 0.2
                        subscribe_test.twist.angular.z = 0.23
                        subscribe_test.vel_pub.publish(subscribe_test.twist)
                        time_distance=current_time - subscribe_test.start_time
               print("time_distance:", time_distance)

            if subscribe_test.data == mode3: 
                 
                found, x_left, x_right = subscribe_test.line_detection(subscribe_test.image)
                angle = 0.0
                speed = 0.0
                if found:
                    x_midpoint = (x_left + x_right) // 2
                    differnce = x_midpoint- View_Center
                    angle = differnce * 0.005
                    speed = 0.42
                    subscribe_test.drive(-angle,speed)
                else :
                    
                    subscribe_test.drive(-angle,speed)
            #print(subscribe_test.twist)

        if drive_mode ==6:
            subscribe_test.twist.linear.x = 0.3
            subscribe_test.check_AR()
            
    rospy.spin()

           
if __name__ == "__main__":
        main()
        rospy.spin()
