#!/usr/bin/python
import sys
import threading
import random as r
import rospy
from std_msgs.msg import Float32, Float64, String

class EyeController(threading.Thread):
    eye_pos = {'far_right': '1:3',
               'right': '2:3',
               'middle': ['3:3','4:3'],
               'left': '5:3',
               'far_left': '6:3'}
    prev = None
    then = 0.0

    def __init__(self):
        self.yaw_sub = rospy.Subscriber('/head/cmd_pose_yaw', Float32, self.callback)
        self.pub = rospy.Publisher('/face', String, queue_size=10)
        threading.Thread.__init__(self)
        self.sleeper = rospy.Rate(10)

    def run(self):
        rospy.loginfo("Jeeves is watching ...")
        while not rospy.is_shutdown():
            dtime = rospy.get_time() - self.then
            if dtime > 2 + r.randint(1, 5):
                self.random_eye()
                self.then = rospy.get_time()
            self.sleeper.sleep()

    def callback(self, data):
        yaw = data.data
        out = 'a'
        now = ''
        if yaw <= -0.6:
            out = self.eye_pos['right']
            now = 'fr'
        elif yaw > -.6 and yaw <= -.4:
            out = self.eye_pos['right']
            now = 'r'
        elif yaw > -.4 and yaw <= .4:
            out = r.choice(self.eye_pos['middle'])
            now = 'm'
        elif yaw > .4 and yaw <= .6:
            out = self.eye_pos['left']
            now = 'l'
        else:
            out = self.eye_pos['left']
            now = 'fl'
        rospy.loginfo("[EyeController]: Got data: {} -- Response {}".format(yaw, out))
        if now != self.prev:
            self.prev = now
            self.pub.publish(String(out))

    def random_eye(self):
        self.pub.publish(String(r.choice(['a','q','w'])))

def main(args):
    rospy.init_node('eye_controller_node', anonymous=True)
    eye = EyeController()
    eye.start()
    rospy.spin()

if __name__ == "__main__":
    main(sys.argv)
