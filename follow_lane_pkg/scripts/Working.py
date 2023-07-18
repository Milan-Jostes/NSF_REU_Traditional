#!/usr/bin/env python3
# Stop at Crosswalk

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from follow_lane_pkg.cfg import FollowLaneConfig   # packageName.cfg
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import numpy as np
#import os
from skimage import data, exposure
from skimage.exposure import match_histograms
import math
empty_msg = Empty()
vel_msg = Twist()
bridge = CvBridge()

def dyn_rcfg_cb(config, level):
  global thresh, drive, speed, ellipse, medBlur, length
  thresh = config.thresh
  speed = config.speed
  drive = config.enable_drive
  ellipse = config.ellipse
  medBlur = config.medBlur
  length = config.length
  return config

def get_centroid(contour):
    M = cv2.moments(contour)
    try:
        cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        return (cx, cy)
    except:
        return None
    
def draw_and_extend_contour(cv_image, contour, color=(255,255,255), thickness=20):
    # Extract the x and y coordinates of the points along the contour
    points = contour.reshape((-1, 2))
    xs = points[:, 0]
    ys = points[:, 1]

    # Fit a second degree polynomial to the points
    coefficients = np.polyfit(xs, ys, 2)
    polynomial = np.poly1d(coefficients)

    # Generate x and y values for the fitted curve
    curve_xs = np.linspace(xs.min(), xs.max(), 500)
    curve_ys = polynomial(curve_xs)

    # Convert the coordinates to integers and zip them into a list of tuples
    curve_points = list(zip(curve_xs.astype(int), curve_ys.astype(int)))

    # Draw the curve on the image
    cv2.polylines(cv_image, [np.array(curve_points)], False, color, thickness)

def draw_and_extend_contour_straight(cv_image, contour, color=(255,255,255), thickness=20):
    # Fit a line to the points in the contour
    [vx, vy, x0, y0] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
    
    # Define a factor to extend the line in both directions
    d = length  # adjust this to your needs
    
    # Calculate start and end points of the extended line
    start = (int(x0 - vx*d), int(y0 - vy*d))
    end = (int(x0 + vx*d), int(y0 + vy*d))
    
    # Draw the extended line
    if (math.atan(abs(end[1]-start[1])/abs(end[0]-start[0]))*180/math.pi)>50:
      cv_image = cv2.line(cv_image, start, end, color, thickness)
    return cv_image


def image_callback(ros_image):
  global bridge, thresh, cols, goal, rate, cols_remove
  rate = rospy.Rate(20)

  try: #convert ros_image into an opencv-compatible imageadi
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
    print(e)

  vel_msg.linear.x = speed
  enable_pub.publish(empty_msg)
  velocity_pub.publish(vel_msg)
  #cv2.imshow('Contours', cv_image)
  cv_image = cv2.resize(cv_image, None, fx=0.7, fy=0.7, interpolation=cv2.INTER_AREA)
  (rows,cols,channels) = cv_image.shape 
  rows_new = int(rows*(1/2))  
  #mid_push = mid
  cols_remove = 0#50
  cv_image = cv_image[rows_new:rows, cols_remove:cols-cols_remove]
  cv_image = cv2.medianBlur(cv_image, medBlur)
  (rows,cols,channels) = cv_image.shape
  #cv2.imshow('Contours', cv_image)
  cv_image = match_histograms(cv_image, goal, channel_axis=-1)

  hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  lower_yellow = np.array([10, 60, 200])
  upper_yellow = np.array([40, 245, 245])
  mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
  kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
  yellow_dilate = cv2.dilate(mask_yellow,kernel2)
  yellow_contours, hierarchy = cv2.findContours(yellow_dilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  cv_image = cv2.drawContours(cv_image, yellow_contours, -1, (0, 0, 0), thickness=5)
  #cv2.imshow("yellow",yellow_img)
  #yellow_detected = cv2.bitwise_and(cv_image, cv_image, mask=mask_yellow)
  #cv_image[np.where((cv_image==yellow_detected).all(axis=2))] = [0,0,0]


  gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
  kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(ellipse,ellipse))

  ret, bw_image = cv2.threshold(gray_image, thresh, 255, cv2.THRESH_BINARY)

 
  dilated = cv2.dilate(bw_image,kernel)
  
  contours, heirarchy = cv2.findContours(dilated.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  #dilated.copy()
  cv_image = cv2.drawContours(cv_image, contours, -1, (255, 255, 255), thickness=8)


  #contours, hierarchy = cv2.findContours(bw_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

  if not contours:
      return
  
  # Initialize two largest areas and contours
  max_c1, max_c2 = None, None
  
  # Sort contours by area in descending order, with the largest contour first.
  sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
#----------------------------------------------------------------------------------
  # Now you can access the i'th largest contour like this:
  cv_image_extended = cv_image.copy()
  if len(sorted_contours) > 1:
#    max_c1 = sorted_contours[0]
#    max_c2 = sorted_contours[1]
#    centroid1 = get_centroid(max_c1)
#    centroid2 = get_centroid(max_c2)
    #i=0
    #sorted_contours = np.array(sorted_contours)
    for i in range(len(sorted_contours)):
      if(i<5 and cv2.contourArea(sorted_contours[i]) > 200):
        max_c = sorted_contours[i]
        cv_image_extended = draw_and_extend_contour_straight(cv_image_extended, max_c)
       #i+=1
    # if centroid1 and centroid2:  # If centroids were calculated correctly
    #     x_distance = abs(centroid1[0] - centroid2[0])
    #     if x_distance < 130 or x_distance > 300:
    #         if len(sorted_contours) > 2:  # If there is a third contour
    #             max_c2 = sorted_contours[2]



  #if max_c1 is not None:
  #    draw_and_extend_contour(cv_image_extended, max_c1)
  #if max_c2 is not None:
  #    draw_and_extend_contour_straight(cv_image_extended, max_c2)

  gray_image_extended = cv2.cvtColor(cv_image_extended, cv2.COLOR_BGR2GRAY)
  ret, bw_image_extended = cv2.threshold(gray_image_extended, thresh, 255, cv2.THRESH_BINARY)
  dilated_extended = cv2.dilate(bw_image_extended,kernel)
  #dilated_extended.copy()
  contours_extended, _ = cv2.findContours(dilated_extended.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  
  if not contours_extended:
    return
  #test_image = cv2.drawContours(cv_image, contours_extended, -1, (0, 255, 0), 3)
  #cv2.imshow('BW', test_image)
  max_area1_extended, max_area2_extended = 0, 0
  max_c1_extended, max_c2_extended = None, None

  # for c in contours_extended:
  #     area = cv2.contourArea(c)
  #     if area > max_area1_extended:
  #         max_area2_extended = max_area1_extended
  #         max_c2_extended = max_c1_extended
  #         max_area1_extended = area
  #         max_c1_extended = c
  #     elif area > max_area2_extended:
  #         max_area2_extended = area
  #         max_c2_extended = c
  areas = [cv2.contourArea(c) for c in contours_extended]
  max_index = np.argmax(areas)
  cnt=contours_extended[max_index]    
  areas[max_index] = 0
  centroid2 = None
  centroid1 = None
  if(len(areas)>1):
    max_index2 = np.argmax(areas)
    cnt2 = contours_extended[max_index2]
    centroid2 = get_centroid(cnt2)
    #test_image2 = cv2.drawContours(cv_image, [cnt,cnt2], -1, (0, 255, 0), 3)
    #cv2.imshow('BW2', test_image2)
  centroid1 = get_centroid(cnt)

  global mid
  if centroid1 is not None and centroid2 is not None:
      mid_cx = (centroid1[0] + centroid2[0]) // 2
      mid_cy = (centroid1[1] + centroid2[1]) // 2
      mid = int(cols / 2)-75  
      cv_image = cv2.line(cv_image_extended, (mid,0), (mid,1000), (255,0,0), 5)
      cv2.circle(cv_image_extended, (mid_cx, mid_cy), 10, (255,0,0), -1)
      drive_to_follow_line(cv_image_extended, mid_cx, mid_cy)
#----------------------------------------------------------------------------------
  # largest_contour = sorted_contours[0]
  # cv_image_extended = draw_and_extend_contour_straight(cv_image.copy(), largest_contour)

  # centroid = get_centroid(largest_contour)

  # if centroid is not None:
  #    cv2.circle(cv_image_extended, centroid, 10, (255,0,0), -1)
  #    drive_to_follow_line(cv_image_extended, centroid[0], centroid[1])
#----------------------------------------------------------------------------------
  cv2.imshow('Contours', cv_image_extended)
  cv2.waitKey(1)
  

def drive_to_follow_line(cv_image, cx, cy):
  global speed, drive,mid
  global mid_temp
  vel_msg.linear.x = speed
  #mid = int(cols / 2)+75  
  if drive == True:
    if cx > mid+15:
      vel_msg.angular.z = 0.8 * (mid-cx)/cx
      velocity_pub.publish(vel_msg)
    elif cx < mid-15:
      vel_msg.angular.z = 0.8 * (mid-cx)/cx
      velocity_pub.publish(vel_msg)
    else:
      vel_msg.angular.z = 0.0
      velocity_pub.publish(vel_msg)
  else:
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    velocity_pub.publish(vel_msg)
  return

if __name__ == '__main__':
  rospy.init_node('follow_lane', anonymous=True)
  imgtopic = rospy.get_param("~imgtopic_name") # private name
  rospy.Subscriber(imgtopic, Image, image_callback)
  enable_pub = rospy.Publisher('/vehicle/enable', Empty, queue_size=1)
  velocity_pub = rospy.Publisher('/vehicle/cmd_vel', Twist, queue_size=1)
  global goal

  goal = cv2.imread("/home/reu-actor/actor_ws/src/follow_lane_pkg/road2.jpg")
  srv = Server(FollowLaneConfig, dyn_rcfg_cb)
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
