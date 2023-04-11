#! /usr/bin/python3
# steve's contour processing
import rospy
import time
import cv2
import numpy as np
from utils import filter_img, find_centroids, sort_points, navigation, locate_leds_while_moving, get_region_of_interest, calc_dist, get_optimal_speed
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from motor_control import Motor


DIMENSION = (240,320) #Height, Width

class TakePhoto:
    def __init__(self):
        self.image_received = False

        img_topic = "/camera/image/compressed"
        self.image_sub = rospy.Subscriber(img_topic, CompressedImage, self.callback, queue_size = 1)

    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    def get_img(self):
        return self.img
    def save_img(self, img_title):
        print(cv2.imwrite(img_title, self.img))

class CmdListener:
    def __init__(self):
        self.cmd = ""
        self.cmd_sub = rospy.Subscriber("/master_cmd", String, self.callback)
    def callback(self, cmd):
        self.cmd = cmd.data
    def get_msg(self):
        return self.cmd
    def setup_order(self):
        while self.get_msg() == "HIGH":
                pass
        print("High ")
        while self.get_msg() == "LOW":
                pass
        print("Low ")


def get_frames_in_batches_on_off(camera_obj, cmd_listener, batch_size = 1):
    on_imgs = np.zeros((batch_size, DIMENSION[0], DIMENSION[1]))
    off_imgs = np.zeros((batch_size, DIMENSION[0], DIMENSION[1]))
    for i in range(batch_size):
        if cmd_listener.get_msg() == "HIGH":
            time.sleep(0.4)
            on_imgs[i] = cv2.cvtColor(camera.get_img(), cv2.COLOR_BGR2GRAY)

            while cmd_listener.get_msg() == "HIGH":
                pass
        if cmd_listener.get_msg() == "LOW":
            time.sleep(0.4)
            off_imgs[i] = cv2.cvtColor(camera.get_img(), cv2.COLOR_BGR2GRAY)

            while cmd_listener.get_msg() == "LOW":
                pass
    return on_imgs, off_imgs

if __name__ == "__main__":
    i = 0
    rospy.init_node("main_auto_drive")
    camera = TakePhoto()
    cmd_listener = CmdListener()
    feedback_pub = rospy.Publisher("/feedback", String, queue_size = 10)
    motor = Motor()
    rate = rospy.Rate(30)
    time.sleep(4)
    while not rospy.is_shutdown():
        if i == 0:
            motor.set_direction("stop")
            feedback_pub.publish(str(0))
            time.sleep(4)
            print("Syncing the LED on-off")
            cmd_listener.setup_order()         #syncronize blinking
            while True:
                on_imgs, off_imgs= get_frames_in_batches_on_off(camera, cmd_listener, 5)

                on_img, off_img, binary_difference = filter_img(on_imgs, off_imgs)

                cnts, _= cv2.findContours(image = binary_difference.astype(np.uint8), mode = cv2.RETR_EXTERNAL, method = cv2.CHAIN_APPROX_SIMPLE)

                centroids = find_centroids(cnts, binary_difference)
                cv2.imwrite("/home/parallels/test_img/on.jpg", on_imgs[0])
                cv2.imwrite("/home/parallels/test_img/off.jpg", off_imgs[0])
                print("Pending " + str(len(centroids)))
                if (len(centroids)) == 3:
                    print("See leds")
                    print("---")
                    break
            centroids = sort_points(centroids)
            print(centroids[0])
            print(centroids[1])
            print(centroids[2])
            distance = calc_dist(centroids)
            print("Distance = ",distance)
            print("------------------------------------------------------------")
            feedback_pub.publish(str(1))
            time.sleep(0.5)
            i+=1
        else:
            on_img = camera.get_img()
            if len(np.unique(on_img)) == 1:
                continue
            cv2.imwrite("/home/parallels/test_img/ori{}.jpg".format(i), on_img)
            centroids = locate_leds_while_moving(on_img, centroids, i)
            if i % 5 == 0:
                print(centroids[0])
                print(centroids[1])
                print(centroids[2])
            if centroids[0] == centroids[1] or centroids[1] == centroids[2] or centroids[0] == centroids[2]:
                i = 0
                continue

            linear_x, angular_z = get_optimal_speed(centroids, i)

            motor.set_direction((linear_x, angular_z))
            distance = calc_dist(centroids)
            if distance <= 15.0:
                print("Stop!")
                motor.set_direction("stop")
                break
            if i % 5 == 0:
                print("Distance: ", end = "");
                print(distance)
                print("------------------------------------------------------------")
            i+=1
        rate.sleep()
