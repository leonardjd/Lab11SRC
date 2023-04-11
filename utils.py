import numpy as np
from PIL import Image
import cv2
import os
import math
import sys

FRAME_DIMENSION = (240,320) #Height and width

def filter_img(on_imgs, off_imgs):
	"""
	This method will return numpy array of led on and led off image
	"""
	on_img_avg = np.average(on_imgs, axis=0)
	off_img_avg = np.average(off_imgs, axis=0)
	difference = on_img_avg - off_img_avg
	binary_difference = ((difference >= 50) * 1).astype(np.float32)

	return on_img_avg, off_img_avg, binary_difference

def find_centroid(area):
	moment = cv2.moments(area)
	centroid_x = int(moment["m10"]/moment["m00"])
	centroid_y = int(moment["m01"]/moment["m00"])

	return (centroid_x, centroid_y)

def find_centroids(cnts, bin_img):
	centroids = []
	for i in range(len(cnts)):
		mask = np.zeros((bin_img.shape[0], bin_img.shape[1], 3), dtype = np.uint8)

		#Get the mask of inner area of each contour
		mask = cv2.drawContours(mask, cnts[i], contourIdx = -1, color=(255,255,255), thickness = cv2.FILLED)

		#Convert the mask to binary format
		mask = cv2.cvtColor(mask, cv2.COLOR_RGB2GRAY)
		#print(mask.shape, np.unique(mask))
		mask = (mask > 200) * 1

		centroids.append(find_centroid(mask.astype(np.uint16)))
	return centroids

def sort_points(points):
	"""
	Return a list where elements at idx0 and idx1 are two front leds, idx2 is back led
	Idea: the front leds will have approxiamtely the same y-coordinates, and their y-coordinates are smaller than the back led's.
	The front led on the left will have smaller x-coordinates than the right front led.
	"""
	sorted_points = sorted(sorted(points, key=lambda point : point[1])[:2]) +  [sorted(points, key=lambda point : point[1])[2]]
	return sorted_points

def navigation(sorted_points):
	"""
	Return:
	- 0 if the robot is in line with the LEDs.
	- 1 if the robot is deviated to the right.
	- -1 if the robot is deviated to the left.
	"""
	#Call A, B, C the back led, left front led, and right front led respectively
	A = sorted_points[2]
	Ax = A[0]
	Ay = A[1]
	B = sorted_points[0]
	C = sorted_points[1]

	if Ax <= (FRAME_DIMENSION[1] / 2 - 2):
		return 1
	elif Ax >= (FRAME_DIMENSION[1] / 2 + 2):
		return -1
	else:
		return 0
def calc_dist(centroids):
	"""
	Return the estimated distance from the robot to the front posts.
	"""
	B = centroids[0]
	C = centroids[1]

	front_leds_dist = calc_dist_between_2_points(B, C)
	#dist = (1.126e4 - front_leds_dist * 13.17) / front_leds_dist
	dist = 2091.17/front_leds_dist -0.53
	return dist

def calc_dist_between_2_points(point1, point2):
	return np.linalg.norm(generate_vector(point1, point2))

def generate_vector(point1, point2):
	return np.array([point2[0] - point1[0], point2[1] - point1[1]])
def locate_leds_while_moving(frame, centroids, i):

	left_region = get_region_of_interest(frame, centroids, "left")
	right_region = get_region_of_interest(frame, centroids, "right")
	back_region = get_region_of_interest(frame, centroids, "back")

	cv2.imwrite("/home/pi/test_img/left_led_{}.jpg".format(i), left_region)
	cv2.imwrite("/home/pi/test_img/right_led_{}.jpg".format(i), right_region)
	cv2.imwrite("/home/pi/test_img/back_led_{}.jpg".format(i), back_region)

	new_left_led = np.where(left_region == np.max(left_region))
	new_right_led = np.where(right_region == np.max(right_region))
	new_back_led = np.where(back_region == np.max(back_region))

	new_left_led = [new_left_led[1][0], new_left_led[0][0]]
	new_right_led = [new_right_led[1][0], new_right_led[0][0]]
	new_back_led = [new_back_led[1][0], new_back_led[0][0]]

	return (new_left_led, new_right_led, new_back_led)

def get_region_of_interest(frame, centroids, position):
	frame_temp = np.copy(frame)

	back_led = centroids[2]
	left_led = centroids[0]
	right_led = centroids[1]

	#Cut the area of interest
	if position == "left":
		x1 = left_led[0] - 30
		x2 = left_led[0] + int(math.floor(0.35 * (back_led[0] - left_led[0])))
		y1 = left_led[1] - 30
		y2 = left_led[1] + 20

	if position == "right":
		x1 = right_led[0] - int(math.floor(0.35 * (right_led[0] - back_led[0])))
		x2 = right_led[0] + 30
		y1 = right_led[1] - 30
		y2 = right_led[1] + 20
		
	if position == "back":
		x1 = back_led[0] - int(math.floor(0.72 * (back_led[0] - left_led[0])))
		x2 = back_led[0] + int(math.floor(0.72 * (right_led[0] - back_led[0])))
		y1 = back_led[1] - 30
		y2 = back_led[1] + 20

	region = frame[y1 : y2, x1 : x2]
	frame_temp.fill(0)
	frame_temp[y1 : y2, x1 : x2] = region

	return frame_temp

def get_optimal_speed(centroids, i):
		#This function will determine the normalized optimal linear and angular speed based on the current location of the led
	COEFFICIENT1 = 0.0004
	COEFFICIENT2 = 0.009

	back_led = centroids[2]
	left_led = centroids[0]
	right_led = centroids[1]

	angular_vel = -(back_led[0] - FRAME_DIMENSION[1]/2) * COEFFICIENT1 - ((left_led[0] + right_led[0])/2.0 - back_led[0]) * COEFFICIENT2
	if i % 5 == 0:
		print("back led with center: " + str(-(back_led[0] - FRAME_DIMENSION[1]/2)))
		print("term1: " + str(-(back_led[0] - FRAME_DIMENSION[1]/2) * COEFFICIENT1))
		print("back led with left and right middle point: " + str(-((left_led[0] + right_led[0])/2.0 - back_led[0])))
		print("term2: " + str(((left_led[0] + right_led[0])/2.0 - back_led[0]) * -COEFFICIENT2))

	if angular_vel > 0.3:
		angular_vel = 0.3
	if angular_vel < -0.3:
		angular_vel = -0.3
		
	linear_vel = 0.085
	if i % 5 == 0:
		print("Linear_vel " + str(linear_vel))
		print("Angular_vel " + str(angular_vel))

	return linear_vel, angular_vel

