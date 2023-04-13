#!/usr/bin/python3

import rospy
from std_msgs.msg import String

global_feedback = 0

def send_cmd(freq):
    rospy.init_node("master_node")
    pub_to_led = rospy.Publisher("/master_cmd", String, queue_size = 10)
    high = False
    while not rospy.is_shutdown():
        if global_feedback == 0:
            if high == True:
                pub_to_led.publish("HIGH")
            else:
                pub_to_led.publish("LOW")
        else:
            pub_to_led.publish("HIGH")
        rospy.sleep(1.0/freq)
        high = not high
def feedback_cb(msg):
    global global_feedback
    global_feedback = int(msg.data)
    print("feedback_cb")

if __name__ == "__main__":
    feedback_listener = rospy.Subscriber("/feedback", String, feedback_cb)
    send_cmd(1)
    rospy.spin()
