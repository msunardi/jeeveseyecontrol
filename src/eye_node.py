#!/usr/bin/python
import sys
import threading
import random as r
import rospy
from std_msgs.msg import Float32, Float64, String
import serial

#face = serial.Serial('/dev/ttyACM0')

class EyeController(threading.Thread):
    eye_pos = {'far_right': '1:3',
               'right': '2:3',
               'middle': ['3:3','4:3'],
               'left': '5:3',
               'far_left': '6:3'}
    eye_yaw = 3
    eye_pitch = 3
    eye_yaw_prev = eye_yaw
    eye_pitch_prev = eye_pitch
    prev = None
    then = 0.0

    def __init__(self):
        #self.yaw_sub = rospy.Subscriber('/head/cmd_pose_yaw', Float32, self.callback)
        self.yaw_sub = rospy.Subscriber('/eye/yaw', Float32, self.yaw_cb)
        self.pitch_sub = rospy.Subscriber('/eye/pitch', Float32, self.pitch_cb)
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

    def yaw_cb(self, data):
        self.eye_yaw = int(data.data)
        out = 'b'
        now = ''

        # Randomness is added so occassionally will update eye position (even when no change)
        # Note: eyes will blink when position value is received and updating.
        if self.eye_yaw != self.eye_yaw_prev or r.random() < 0.2:
            self.eye_yaw_prev = self.eye_yaw
            out = "{}:{}".format(self.eye_yaw, self.eye_pitch)
            rospy.loginfo("[EyeController]: Got yaw data: {} -- Response {}".format(self.eye_yaw, out))
            self.pub.publish(String(out))
        else:
            rospy.loginfo("[EyeController]: No new yaw")

    def pitch_cb(self, data):
        self.eye_pitch = int(data.data)
        out = 'b'
        now = ''
        # Randomness is added so occassionally will update eye position (even when no change)
        if self.eye_pitch != self.eye_pitch_prev or r.random() < 0.2:
            self.eye_pitch_prev = self.eye_pitch
            out = "{}:{}".format(self.eye_yaw, self.eye_pitch)
            rospy.loginfo("[EyeController]: Got pitch data: {} -- Response {}".format(self.eye_pitch, out))
            self.pub.publish(String(out))
        else:
            rospy.loginfo("[EyeController]: No new pitch")

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
            #if face.isOpen():
            #    try:
            #        face.write(str(out))
            #    except Exception as e:
            #        rospy.logerr('face.write failed: {}'.format(e))

    def random_eye(self):
        self.pub.publish(String(r.choice(['a','q','w'])))

def main(args):
    rospy.init_node('eye_controller_node', anonymous=True)
    eye = EyeController()
    eye.start()
    rospy.spin()

if __name__ == "__main__":
    main(sys.argv)
    #if face.isOpen():
    #    face.close()
