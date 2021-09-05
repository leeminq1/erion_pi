#!/usr/bin/python
"""
sonar get distance

"""
import math
import time
import rospy
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

DIST_STEER_ENGAGE = 1.2
DIST_BREAK = 0.4

DIST_LAT_ENGAGE = 0.4

K_FRONT_DIST_TO_SPEED = 1.0
K_LAT_DIST_TO_STEER = 2.0

TIME_KEEP_STEERING = 1.5


def saturate(value, min, max):
    if value <= min:
        return(min)
    elif value >= max:
        return(max)
    else:
        return(value)


class ObstAvoid():
    def __init__(self):

        # initial value
        # sonar
        self.range_center = 3
        self.range_left = 3
        self.range_right = 3

        # auto_drive
        self._time_steer = 0
        self._steer_sign_prev = 0

        # camera
        self.obj_arr = []

        # Subscriber
        #-------------------- sonar -------------------------#
        self.sub_center = rospy.Subscriber(
            "/dkcar/sonar/1", Range, self.update_range)
        self.sub_left = rospy.Subscriber(
            "/dkcar/sonar/0", Range, self.update_range)
        self.sub_right = rospy.Subscriber(
            "/dkcar/sonar/2", Range, self.update_range)
        # rospy.loginfo("Sonar Subscribers set")

        # sonar update func
    def update_range(self, message):
        angle = message.field_of_view

        if abs(angle) < 0.1:
            self.range_center = message.range

        elif angle > 0:
            self.range_right = message.range

        elif angle < 0:
            self.range_left = message.range

        rospy.loginfo("Sonar array: %.1f  %.1f  %.1f" %
                      (self.range_left, self.range_center, self.range_right))

       #-------------------- camera -------------------------#

        self.sub_camera = rospy.Subscriber(
            "detections", Detection2DArray, self.update_object)

    # camera update func
    def update_object(self, message):
       # structure example
        '''
         Detection2DArray  : {
             header
             detections : { 
                 result : { id, score, pose}
                 bbox : {center : { x,y,theta}, size_x, size_y},
                 source_img : { header, height , width , encoding , is_bigendian , step , data } 
             }
          }
        '''
        # msg destructure
        detections = message.detections
        # detections detections
        id = detections.result.id
        score = detections.result.score
        bbox_size_x = detections.bbox.size_x
        bbox_size_y = detections.bbox.size_y
        bbox_x = detections.bbox.center.x
        bbox_y = detections.bbox.center.y
        # set array = [id, score , size_x, size_y, x,y]
        self.obj_arr = [id, score, bbox_size_x, bbox_size_y, bbox_x, bbox_y]
        # subscireber value info
        rospy.loginfo('Obj_info={},{},{},{},{},{}'.format(id,score,bbox_size_x,bbox_size_y,bbox_x,bbox_y))

       # pusblish
        self.pub_twist = rospy.Publisher(
            "/auto_drive_control/cmd_vel", Twist, queue_size=5)
        # rospy.loginfo("Publisher set")

        self._message = Twist()

    def test_run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # -- Get the control action
            if self.obj_arr[0] == 0:
                self._message.linear.x = 1
                self._message.angular.z = 0
                if self.range_center < 5:
                    self._message.linear.x = 0
                    self._message.angular.z = 0

            # -- publish it
            self.pub_twist.publish(self._message)

            rate.sleep()

    # def get_control_action(self):
    #     """
    #     Based on the current ranges, calculate the command
    #     """

    #     break_action = 1.0
    #     steer_action = 0.0

    #     # --- Get the minimum distance
    #     range = min([self.range_center, self.range_left, self.range_right])
    #     # print "%.2f    %.2f  %.2f  %.2f"%(range,  self.range_left, self.range_center, self.range_right)

    #     if self.range_center < DIST_STEER_ENGAGE:
    #         # --- Start applying the break
    #         # break_action   = (range - DIST_BREAK)/(DIST_STEER_ENGAGE - DIST_BREAK)
    #         adim_dist = range/DIST_STEER_ENGAGE
    #         if range < DIST_BREAK:
    #             break_action = K_FRONT_DIST_TO_SPEED*(range/DIST_BREAK)
    #             break_action = saturate(break_action, 0, 1)
    #             rospy.loginfo("Engaging break %.1f" % break_action)

    #         # --- Apply steering, proportional to how close is the object
    #         steer_action = K_LAT_DIST_TO_STEER*(1.0 - adim_dist)
    #         steer_action = self.get_signed_steer(steer_action)

    #         steer_action = saturate(steer_action, -1.5, 1.5)
    #         rospy.loginfo("Steering command %.2f" % steer_action)

    #     return (break_action, steer_action)

    # def get_signed_steer(self, steer_action):

    #     if time.time() > self._time_steer + TIME_KEEP_STEERING:
    #         print("> Update steer_action sign")
    #         self._time_steer = time.time()

    #         # -- If the lwft object is closer, turn right (negative)
    #         if self.range_left < self.range_right:
    #             steer_action = -steer_action

    #         if steer_action >= 0:
    #             self._steer_sign_prev = 1
    #         else:
    #             self._steer_sign_prev = -1

    #     else:
    #         steer_action *= self._steer_sign_prev

    #     return (steer_action)

    # def run(self):

    #     # --- Set the control rate
    #     rate = rospy.Rate(5)

    #     while not rospy.is_shutdown():
    #         # -- Get the control action
    #         break_action, steer_action = self.get_control_action()

    #         # rospy.loginfo("Throttle = %3.1f    Steering = %3.1f"%(break_action, steer_action))

    #         # -- update the message
    #         self._message.linear.x = break_action
    #         self._message.angular.z = steer_action

    #         # -- publish it
    #         self.pub_twist.publish(self._message)
    #         rate.sleep()
if __name__ == "__main__":

    rospy.init_node('obstacle_avoid')

    obst_avoid = ObstAvoid()
    # obst_avoid.run()
