#!/usr/bin/env python3
# Stop at Crosswalk

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from follow_lane_pkg.cfg import FollowLaneConfig   # packageName.cfg
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Empty
import numpy as np
#import os
from skimage import data, exposure
from skimage.exposure import match_histograms
import math
empty_msg = Empty()
vel_msg = Twist()
bridge = CvBridge()


def find_centroids(temp):
  global cols, last1, last2
  mid = int(cols / 2)#+50#-75  
  dist = []
  left = []
  right = []
  centroids = []
  for val in temp:
    if val != None :
        centroids.append(val)
  for x in centroids:
    dist.append(abs(x[0]-mid + -x[1]))
  max1 = centroids[np.argmin(dist)]
  dist[np.argmin(dist)] = 100000
  max2 = centroids[np.argmin(dist)]
  numl=0
  numr = 0
  left=[]
  right = []
  if last1 and last2:
     for x in centroids:
      left.append(abs(abs(x[0]-last1[0])-mid )+ -x[1])
      right.append(abs(abs(x[0]-last2[0])-mid)+ -x[1])
      max1 = centroids[np.argmin(left)]
      max2 = centroids[np.argmin(right)]

  # if left and right:  
  #   if np.argmin(left)>1000:
  #     centroids[np.argmin(left)] = (1900,370)
  #   elif np.argmin(right)>1000:
  #     centroids[np.argmin(right)] = (1900,370)
  #   max1 = centroids[np.argmin(left)]
  #   max2 = centroids[np.argmin(right)]
  #max1=None
  #max2= None
  #  for x in centroids:
  #    #dist[x] = (x[0]-mid)
  #    if x[0]-mid <0:
  #      left.append(abs(x[0]-mid))
  #      numl =1
  #    else:
  #      right.append(x[0]-mid)
  #      numr = 1
  #  if numl==1:
  #    l = left[np.argmin(left)]
  #    for z in centroids:
  #    #dist[x] = (x[0]-mid)
  #      if abs(z[0]-mid) == l:
  #        max1 = z#Here is the issue  
  #  #dist[np.argmin(dist)] = 10000
  #  if numr ==1:
  #    r = right[np.argmin(right)]
  #    for z in centroids:
  #    #dist[x] = (x[0]-mid)
  #      if z[0]-mid == r:
  #        max2 = z #Here is the issue 
  #    #max2 = centroids[np.argmin(right)]
  # for x in centroids:
  #   dist.append(abs(x[0]-mid))
  # dist1 = centroids[np.argmin(dist)]
  # dist[np.argmin(dist)] = 10000
  # dist2 = centroids[np.argmin(dist)]
  # if max1 == dist1 and max2 == dist2:
  #   return max1,max2
  # elif max1==dist1 or max1==dist2:
  #   return max1,None
  # elif max2==dist2 or max2 == dist1:
  #   return max2,None
  
  # dist_con = sorted(centroids, key=lambda centroids: centroids[0], reverse=False)
  # middleIndex = (len(dist_con) - 1)/2
  # if(middleIndex%1)==0.5:
  #   max1 = centroids[int(middleIndex-0.5)]
  #   max2 = centroids[int(middleIndex+0.5)]
  # else:
  #   max1 = centroids[int(middleIndex)]
  #   temp1 = centroids[int(middleIndex-1)]
  #   temp2 = centroids[int(middleIndex+1)]
  #   if max1[0]<(mid+20):
  #     max2 = temp1
  #   else:
  #     max2 = temp2
  # left_dist = []
  # right_dist = []
  # dist_right = []
  # dist_left = []
  # if last1 is not None and last2 is not None:
  #   print("test")
  #   rightline = (1250,250)
  #   left_x = last1[0]
  #   right_x = last2[0]
  #   for x in centroids:
  #     dist_right.append(abs(x[0]-rightline[0]+x[1]-rightline[1]))
      
  #   max1 = centroids[np.argmin(dist_right)]
  #   find = (max1[0]-700,max1[1])
  #   for x in centroids:
  #     dist_left.append(abs(x[0]-find[0]+x[1]-find[1]))
  #   max2 = centroids[np.argmin(dist_left)]
#       y_dif_l = (x[1] - last1[1])
#       y_dif_r = (x[1] - last2[1])
# #      if x[1] >100:
#       left_dist.append(abs(x[0] - left_x-y_dif_l))
#       right_dist.append(abs(x[0] - right_x-(y_dif_r)))
      
#       dist_mid.append(abs(x[0]-mid))
#      else:
#        left_dist.append(10000)
#        right_dist.append(10000)
      
    #max1=centroids[np.argmin(left_dist)]
    #max2=centroids[np.argmin(right_dist)]
#     limit = 0
# #
#     max1 = centroids[np.argmin(dist_mid)]
#     dist_mid[np.argmin(dist_mid)] = 10000
#     max2 = centroids[np.argmin(dist_mid)]

# #

#     while limit <5:
#       if abs(max1[0]-max2[0]) <450:
#         dist_mid[np.argmin(dist_mid)] = 10000
#         max2 = centroids[np.argmin(dist_mid)]
#       limit=limit+1

         
    #     if np.argmin(left_dist)<np.argmin(right_dist):
    #       right_dist[np.argmin(right_dist)]=10000
    #       max2=centroids[np.argmin(right_dist)]
    #     elif np.argmin(left_dist)>np.argmin(right_dist):
    #       left_dist[np.argmin(left_dist)]=10000
    #       max2=centroids[np.argmin(left_dist)]
    # # while abs(max1[0]-max2[0]) <150 and limit <5:
    # #   right_dist[np.argmin(right_dist)] = 10000
    # #   max2=centroids[np.argmin(right_dist)]
    #   limit = limit+1
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
#    max1=centroids[np.argmin(dist_mid)]
#    dist_mid[np.argmin(dist_mid)] = 10000
#    max2=centroids[np.argmin(dist_mid)]
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    #200 200 1000 200

    
    #max2=centroids[np.argmin(right_dist)]
  
  
  last1 = max1
  if max1 != max2:
    last2 = max2
  return max1,max2



def dyn_rcfg_cb(config, level):
  global thresh, drive, speed, ellipse, medBlur, length,blur_shadow, ellipse_shadow, thresh_shadow, gamma, alpha, beta
  thresh = config.thresh
  speed = config.speed
  drive = config.enable_drive
  ellipse = config.ellipse
  medBlur = config.medBlur
  length = config.length
  blur_shadow = config.blur_shadow
  ellipse_shadow = config.ellipse_shadow
  thresh_shadow = config.thresh_shadow
  gamma = config.gamma
  alpha = config.alpha
  beta = config.beta
  return config

# def find_centroids(temp):
#   global cols
#   mid = int(cols / 2)-75  
#   dist = []
#   centroids = []
#   for val in temp:
#     if val != None :
#         centroids.append(val)
#   for x in centroids:
#     dist.append(abs(x[0]-mid))
#   max1 = centroids[np.argmin(dist)]
#   dist[np.argmin(dist)] = 10000
#   max2 = centroids[np.argmin(dist)]
#   return max1,max2

def get_centroid(contour):
    M = cv2.moments(contour)
    try:
        cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        return (cx, cy)
    except:
        return None
    
# def draw_and_extend_contour(cv_image, contour, color=(255,255,255), thickness=20):
#     # Extract the x and y coordinates of the points along the contour
#     points = contour.reshape((-1, 2))
#     xs = points[:, 0]
#     ys = points[:, 1]

#     # Fit a second degree polynomial to the points
#     coefficients = np.polyfit(xs, ys, 2)
#     polynomial = np.poly1d(coefficients)

#     # Generate x and y values for the fitted curve
#     curve_xs = np.linspace(xs.min(), xs.max(), 500)
#     curve_ys = polynomial(curve_xs)

#     # Convert the coordinates to integers and zip them into a list of tuples
#     curve_points = list(zip(curve_xs.astype(int), curve_ys.astype(int)))

#     # Draw the curve on the image
#     cv2.polylines(cv_image, [np.array(curve_points)], False, color, thickness)

def draw_and_extend_contour_straight(cv_image, contour, color=(255,255,255), thickness=20):
    # Fit a line to the points in the contour
    if cv2.contourArea(contour)>200:
      [vx, vy, x0, y0] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
      
      # Define a factor to extend the line in both directions
      d = length  # adjust this to your needs
      
      # Calculate start and end points of the extended line
      start = (int(x0 - vx*d), int(y0 - vy*d))
      end = (int(x0 + vx*d), int(y0 + vy*d))
      
      # Draw the extended line
      if(abs(end[0]-start[0])!=0):
        if abs(math.atan(abs(end[1]-start[1])/abs(end[0]-start[0]))*180/math.pi)>5:#
          cv_image = cv2.line(cv_image, start, end, color, thickness)
    return cv_image

# def remove_shadows(img):
#   rgb_planes = cv2.split(img)
#   result =[]
#   result_norm = []
#   for plane in rgb_planes:
#     dilated = cv2.dilate(plane, np.ones((7,7),np.uint8))
#     bg_img = cv2.medianBlur(dilated, 21)
#     diff = 255 -cv2.absdiff(plane, bg_img)
#     norm = cv2.normalize(diff, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
#     result.append(diff)
#     result_norm.append(norm)

#     imgOut = cv2.merge(result)
#     imgOut2 = cv2.merge(result_norm)

#   return imgOut2


def preprocess(img):
    img_original = np.copy(img)
    global blur_shadow,ellipse_shadow,thresh_shadow,gamma,alpha,beta
    medBlur = blur_shadow
    ellipse = ellipse_shadow
    thresh = thresh_shadow    
    gamma =gamma
    alph=alpha
    bet=beta
    cv_image = cv2.medianBlur(img, medBlur)
    (rows,cols,channels) = cv_image.shape
    #cv2.imshow('Contours', cv_image)
    goal = cv2.imread("/home/milan/catkin_ws/src/follow_lane_pkg/road2.jpg")
    #cv_image = match_histograms(cv_image, goal, channel_axis=-1)
    # div =  8
    # quantized = img // div * div + div // 2

    # cv2.imshow('quantize', quantized)
    # Convert to LAB color space


    # Show img_contrast as output
    #cv2.imshow('img_contrast', img_contrast_uint8)
    
    
    
    
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([10, 60, 200])
    upper_yellow = np.array([40, 245, 245])
    mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    yellow_dilate = cv2.dilate(mask_yellow,kernel2)
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
    red_contours, hierarchy = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    yellow_contours, hierarchy = cv2.findContours(yellow_dilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    #cv_image = cv2.drawContours(cv_image, red_contours, -1, (255, 255, 0), thickness=5)
    #cv_image = cv2.drawContours(cv_image, yellow_contours, -1, (0, 0, 0), thickness=5)
    #cv2.imshow("yellow",yellow_img)
    #yellow_detected = cv2.bitwise_and(cv_image, cv_image, mask=mask_yellow)
    #cv_image[np.where((cv_image==yellow_detected).all(axis=2))] = [0,0,0]
    if red_contours:
        red_area = [cv2.contourArea(c) for c in red_contours]
        max = np.argmax(red_area)
        max_red = red_contours[max]
        #if cv2.contourArea(max_red)>100:
        #print("STOP")

    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
    
    
    
    
    
    
    # global block,cVal
    # blurred = cv2.medianBlur(gray_image,25)#cv2.GaussianBlur(gray_image, (5, 5), 0)
    # threshadapt = cv2.adaptiveThreshold(blurred, 255,
        # cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, block, cVal)
    # cv2.imshow("Mean Adaptive Thresholding", threshadapt)
    blur = cv2.GaussianBlur(gray_image,(5,5),0)
    ret3,th3 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(20,20))
    #dilate_otsu =cv2.erode(th3,kernel)
    #cv2.imshow("Otsu", th3) 
    #test = cv2.findContours(th3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #temp = cv2.drawContours(th3, test, -1, (0, 255, 0), thickness=10)
    #cv2.imshow("Cont", temp)
    
    
    # lines = cv2.HoughLinesP(th3, 
    #                     rho=1, 
    #                     theta=0.01745329251, 
    #                     threshold=100,
    #                     minLineLength=70,
    #                     maxLineGap=4
    #                     )
    
    # if lines is not None:
    #     lines=[l[0] for l in lines]
    #     slopes=[]
    #     for l in lines:
    #         # Graph lines on proc_image
    #         # (l[0],l[1]),(l[2],l[3]) are start and end point respectively
    #         # (255,0,0) is color of line(blue)
    #         # 2 is thickness of line
    #         slope=0
    #         try:
    #             slope=(l[1]-l[3])/(l[0]-l[2])
    #         except:
    #             #rospy.logwarn("Divided by zero in slopes")
    #             continue
    #         if abs(slope)<.5 or abs(slope)>100:
    #             continue
            
    #         cv2.line(image,(l[0],l[1]),(l[2],l[3]),(255,0,0),2)
    # cv2.imshow("Lines",image)
    # ret, bw_image = cv2.threshold(th3, 0, 125, cv2.THRESH_BINARY)
    
    # contours, heirarchy = cv2.findContours(bw_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # mask = np.zeros(th3.shape, np.uint8)
    # mask = cv2.drawContours(mask, contours, -1, (255,255,0), -1)
    # mean = cv2.mean(th3, mask=mask)
    # cv2.imshow(str(mean),mask)
    
    
    # ret, bw_image = cv2.threshold(th3, 0, 125, cv2.THRESH_BINARY)
    # bw_image = cv2.dilate(bw_image,kernel)
    # contours, heirarchy = cv2.findContours(bw_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # dilate_otsu =cv2.dilate(th3,kernel)
    # test = img.copy()
    # test = cv2.drawContours(img,contours,-1,(0,255,0),10)
    # cv2.imshow("Otsu Inv",test)

    th3 = cv2.bitwise_not(th3)
    ret, bw_image = cv2.threshold(th3, 0, 125, cv2.THRESH_BINARY)
    bw_image = cv2.dilate(bw_image,kernel)
    contours, heirarchy = cv2.findContours(bw_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    dilate_otsu =cv2.dilate(th3,kernel)
    erode_otsu = cv2.erode(th3,kernel)
    test = img.copy()
    # contours_find = cv2.bitwise_not(bw_image)
    # contours_not, heirarchy = cv2.findContours(contours_find, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # contours_not = contours_not - contours
    #black_canvas = np.zeros_like(img)
    #cv2.drawContours(black_canvas, contours, -1, 255, cv2.FILLED)
    #cv2.imshow("Contours only",black_canvas)
    #test = cv2.drawContours(test,contours_not,-1,(0,0,0),10)
    
    #convex_contours=[]
    

    #for contour in contours:
    #    convex = cv2.convexHull(contour)
    #    convex_contours.append(convex)
    #test = cv2.dilate(test,kernel)
    test = cv2.drawContours(test,contours,-1,(0,0,0),-1)
    test = cv2.cvtColor(test, cv2.COLOR_BGR2GRAY)
    ret, bw_image = cv2.threshold(test, 1, 255, cv2.THRESH_BINARY)
    contours_not,hierarchy  = cv2.findContours(test, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    test = cv2.drawContours(test,contours_not,-1,(255,255,255),-1)
    
    
    mask = np.zeros(test.shape, np.uint8)
    
    temp = cv2.drawContours(mask, contours, -1, 255, -1)
    mean = cv2.mean(img, mask=mask)    
    mask2 = np.zeros(test.shape, np.uint8)
    temp2 = cv2.drawContours(mask2, contours_not, -1, 255, -1)
    mean2 = cv2.mean(img,mask=mask2)
    #cv2.imshow(str(mean),temp)
    #cv2.imshow(str(mean2),temp2)
    b = mean2[0] - mean[0]
    g = mean2[1] - mean[1]
    r = mean2[2] - mean[2]
    print(b,g,r)
    no_shadow = np.copy(img_original)
    # boolean indexing and assignment based on mask
    #
    _, mask_use = cv2.threshold(temp, thresh=180, maxval=255, type=cv2.THRESH_BINARY)
    no_shadow[(mask_use==255)] = [255,255,255]
    no_shadow_img = cv2.addWeighted(no_shadow, 0.3, img, 0.7, 0, no_shadow)
    #cv2.imshow("Shadow Gone",no_shadow)

    con = np.copy(img_original)
    con_test = cv2.convertScaleAbs(con,alpha=alph,beta=bet)
    lookUp = np.empty((1,256),np.uint8)
    for i in range(256):
        lookUp[0,i] = np.clip(pow(i/255.0,gamma)*255.0,0,255)
    res = cv2.LUT(img_original, lookUp)
    con_test = cv2.LUT(con_test, lookUp)
    #cv2.imshow("Con Table",res)
    #cv2.imshow("Con man", con_test)
    
    
    
    mask = np.zeros(test.shape, np.uint8)
    
    temp = cv2.drawContours(mask, contours, -1, 255, -1)
    mean = cv2.mean(img, mask=mask)    
    mask2 = np.zeros(test.shape, np.uint8)
    temp2 = cv2.drawContours(mask2, contours_not, -1, 255, -1)
    mean2 = cv2.mean(img,mask=mask2)
    #cv2.imshow(str(mean),temp)
    #cv2.imshow(str(mean2),temp2)
    b = mean2[0] - mean[0]
    g = mean2[1] - mean[1]
    r = mean2[2] - mean[2]
    print(b,g,r)
    no_shadow = np.copy(img_original)
    # boolean indexing and assignment based on mask
    #
    _, mask_use = cv2.threshold(temp, thresh=180, maxval=255, type=cv2.THRESH_BINARY)
    no_shadow[(mask_use==255)] = [255,255,255]
    no_shadow_img = cv2.addWeighted(no_shadow, 0.3, img, 0.7, 0, no_shadow)
    #cv2.imshow("Shadow Gone",no_shadow)
    
    
    
    
    
    
    makeimg = res
    #no_shadow
    gray_image = cv2.cvtColor(makeimg, cv2.COLOR_BGR2GRAY)
    
    
    
    #gray_image = cv2.drawContours(gray_image, contours, -1, (0, 0, 0), thickness=10)
    
    
    

    
    edges = cv2.Canny(res,80,255)
    edges = cv2.drawContours(edges, contours_not, -1, (0,0,0),-1)
    edges = cv2.drawContours(edges, contours_not, -1, (0,0,0),12)
    contours, heirarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    gray_image = cv2.drawContours(gray_image,contours,-1,(255,255,255),4)
    #cv2.imshow("edge",edges)
    
    


    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(ellipse,ellipse))
    
    ret, bw_image = cv2.threshold(gray_image, thresh, 255, cv2.THRESH_BINARY)

    dilated = cv2.dilate(bw_image,kernel)
    
    contours, heirarchy = cv2.findContours(dilated.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  #dilated.copy()
    #cv2.imshow("Gray",dilated)
    contour_sized = [c for c in contours if cv2.contourArea(c)>400]
    
    cv_image = cv2.drawContours(makeimg, contour_sized, -1, (255, 255, 255), thickness=-1)
    return cv_image

def velocity_callback(msg):
  enable_pub.publish(empty_msg)


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
  rows_new = int(rows*(5/8))  
  #mid_push = mid
  cols_remove = 0#50
  cv_image = cv_image[rows_new:rows, cols_remove:cols-cols_remove]
  #cv_image = remove_shadows(cv_image)
 # cv_image = cv2.BackgroundSubtractorMOG2(cv_image)
  # cv_image = cv2.medianBlur(cv_image, medBlur)
  # (rows,cols,channels) = cv_image.shape
  # #cv2.imshow('Contours', cv_image)
  # cv_image = match_histograms(cv_image, goal, channel_axis=-1)
  # hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  # lower_yellow = np.array([10, 60, 200])
  # upper_yellow = np.array([40, 245, 245])
  # mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
  # kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
  # yellow_dilate = cv2.dilate(mask_yellow,kernel2)
  # yellow_contours, hierarchy = cv2.findContours(yellow_dilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  # cv_image = cv2.drawContours(cv_image, yellow_contours, -1, (0, 0, 0), thickness=5)
  # #cv2.imshow("yellow",yellow_img)
  # #yellow_detected = cv2.bitwise_and(cv_image, cv_image, mask=mask_yellow)
  # #cv_image[np.where((cv_image==yellow_detected).all(axis=2))] = [0,0,0]

  cv_image = preprocess(cv_image)
  gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
  kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(ellipse,ellipse))

  ret, bw_image = cv2.threshold(gray_image, thresh, 255, cv2.THRESH_BINARY)

 
  dilated = cv2.dilate(bw_image,kernel)
  
  contours, heirarchy = cv2.findContours(dilated.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  #dilated.copy()
  contourDraw = [c for c in contours if cv2.contourArea(c)>200]
  cv_image = cv2.drawContours(cv_image, contourDraw, -1, (255, 255, 255), thickness=8)


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
  if len(sorted_contours) >= 1:
#    max_c1 = sorted_contours[0]
#    max_c2 = sorted_contours[1]
#    centroid1 = get_centroid(max_c1)
#    centroid2 = get_centroid(max_c2)
    #i=0
    #sorted_contours = np.array(sorted_contours)
    for i in range(len(sorted_contours)):
      if(i<5 and cv2.contourArea(sorted_contours[i]) > 250):
        rospy.logwarn(f"contour {i} area = {cv2.contourArea(sorted_contours[i])}")
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
  #corner = 100
  #dilated_extended = cv2.line(dilated_extended, (0,corner), (corner,0), (0,0,0), 100)
  contours_extended, _ = cv2.findContours(dilated_extended.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  contours_extended = [c for c in contours_extended if cv2.contourArea(c)>200]
  cv_image_extended = cv2.drawContours(cv_image_extended, contours_extended, -1, (0, 0, 255), thickness=8)

  
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
  centroid1 = None
  centroid2 = None
  centroid3 = None
  centroid4 = None 
  #last1 = None
  #last2 = None
  centroid1 = get_centroid(cnt)
  if(len(areas)>1):
    max_index2 = np.argmax(areas)
    cnt2 = contours_extended[max_index2]
    centroid2 = get_centroid(cnt2)
    areas[max_index2] = 0
    if(len(areas)>2):
      max_index3 = np.argmax(areas)
      cnt3 = contours_extended[max_index3]
      centroid3 = get_centroid(cnt3)
      areas[max_index3] = 0
      if(len(areas)>3):
        max_index4 = np.argmax(areas)
        cnt4 = contours_extended[max_index4]
        centroid4 = get_centroid(cnt4)
        
  centroid1,centroid2 = find_centroids([centroid1,centroid2,centroid3,centroid4])
  print("C1: ",centroid1)
  print("C2: ",centroid2)
  cv2.circle(cv_image_extended, (centroid1[0], centroid1[1]), 10, (0,255,0), -1)
  cv2.circle(cv_image_extended, (centroid2[0], centroid2[1]), 10, (0,255,0), -1)

  global mid
  if centroid1 is not None and centroid2 is not None:
      mid_cx = (centroid1[0] + centroid2[0]) // 2
      mid_cy = (centroid1[1] + centroid2[1]) // 2
      mid = int(cols / 2)#+50#-75  
      cv_image = cv2.line(cv_image_extended, (mid,0), (mid,1000), (255,0,0), 5)
      cv2.circle(cv_image_extended, (mid_cx, mid_cy), 10, (255,0,0), -1)
      drive_to_follow_line(cv_image_extended, mid_cx)
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
  

def drive_to_follow_line(cv_image, cx):
  global speed, drive,mid
  global mid_temp
  vel_msg.linear.x = speed
  #mid = int(cols / 2)+75  
  if drive == True:
    if abs(cx-mid) > 15:
      vel_msg.angular.z = 0.8 * (mid-cx)/cx
      if vel_msg.angular.z <-0.15:
         print("CORNER*****************************************************************")
         vel_msg.angular.z =vel_msg.angular.z *2.9*(1+math.log(speed))
      if vel_msg.angular.z <-0.17:
         vel_msg.angular.z =vel_msg.angular.z *1.2
         
      velocity_pub.publish(vel_msg)
    else:
      vel_msg.angular.z = 0.0
      velocity_pub.publish(vel_msg)
  else:
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    velocity_pub.publish(vel_msg)
  print("Turn",vel_msg.angular.z)
  return

if __name__ == '__main__':
  rospy.init_node('follow_lane', anonymous=True)
  imgtopic = rospy.get_param("~imgtopic_name") # private name
  rospy.Subscriber('/vehicle/twist', TwistStamped, velocity_callback)
  rospy.Subscriber(imgtopic, Image, image_callback)
  enable_pub = rospy.Publisher('/vehicle/enable', Empty, queue_size=1)
  velocity_pub = rospy.Publisher('/vehicle/cmd_vel', Twist, queue_size=1)
  global goal, last1, last2
  last1 = None
  last2 = None

  goal = cv2.imread("/home/reu-actor/actor_ws/src/follow_lane_pkg/road2.jpg")
  srv = Server(FollowLaneConfig, dyn_rcfg_cb)
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
