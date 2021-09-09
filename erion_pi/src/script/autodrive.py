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
from std_msgs.msg import Int16MultiArray

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


class AutoDrive():
    def __init__(self):

        # initial value
        # sonar
        self.range_center = 6
        self.range_left = 6
        self.range_right = 6

        # auto_drive
        self._time_steer = 0
        self._steer_sign_prev = 0

        # camera
        self.obj_arr = []

        # Subscriber
        # ------------------mode info ----------------------#
        self.mode_sub = rospy.Subscriber(
            "roscar_teleop_cmd_vel", Int16MultiArray, self.update_mode)

        #-------------------- sonar -------------------------#
        self.sub_center = rospy.Subscriber(
            "/dkcar/sonar/1", Range, self.update_range)
        self.sub_left = rospy.Subscriber(
            "/dkcar/sonar/0", Range, self.update_range)
        self.sub_right = rospy.Subscriber(
            "/dkcar/sonar/2", Range, self.update_range)
        rospy.loginfo("Sonar Subscribers set")

        #-------------------- camera -------------------------#

        self.sub_camera = rospy.Subscriber(
            "/detectnet/detections", Detection2DArray, self.update_object)

       # pusblish
        self.pub_auto_cmd = rospy.Publisher(
            "/auto_drive_control/cmd_vel", Int16MultiArray, queue_size=5)
        rospy.loginfo("Publisher set")
        self._message = Int16MultiArray()
        self._message.data = [0, 0, 0]

        rospy.spin()

        # sonar update func

    def update_mode(self, message):
        global f_autodrive
        f_autodrive = message.data[2] == 0
        if f_autodrive:
            rospy.loginfo("Auto_drive mode set")

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

    # camera update func

    def update_object(self, message):
       # structure example
        '''
         Detection2DArray  : [
             header
             detections : { 
                 result : { id, score, pose}
                 bbox : [center : { x,y,theta}, size_x, size_y]
                 source_img : { header, height , width , encoding , is_bigendian , step , data } 
             }
          ]
        '''
        # msg destructure
        detections = message.detections
       # print(detections)
        # detections detections
        id = detections[0].results[0].id
        score = detections[0].results[0].score
        bbox_size_x = detections[0].bbox.size_x
        bbox_size_y = detections[0].bbox.size_y
        bbox_x = detections[0].bbox.center.x
        bbox_y = detections[0].bbox.center.y
        # set array = [id, score , size_x, size_y, x,y]
        self.obj_arr = [id, score, bbox_size_x, bbox_size_y, bbox_x, bbox_y]
        # subscireber value info
        rospy.loginfo(
            '-----------------------------------------------------------')
        rospy.loginfo('id : {}, score : {}, box_size_x : {}, box_size_y :{} , box_x : {}, box_y : {}'.format(
            id, score, bbox_size_x, bbox_size_y, bbox_x, bbox_y))

        # detect condition
        global f_person
        f_person = self.obj_arr[0] == 1
        # while loop command
        # we must break to reset f_person value
        if f_person and f_autodrive:
            self.test_run()
        else:
            rospy.loginfo("Not working")

    def test_run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # -- Get the control action
            self._message.data[0] = 1
            self._message.data[1] = 0
            self.pub_auto_cmd.publish(self._message)
            rospy.loginfo('vehicle go!!!,  accel:{}, streer:{}'.format(
                self._message.data[0], self._message.data[1]))
            if self.range_center < 5:
                self._message.data[0] = 0
                self._message.data[1] = 0
                self.pub_auto_cmd.publish(self._message)
                rospy.loginfo('Object close ! vehicle stop!!')
                break
            elif self.range_left < 5 and self.range_center > 5:
                self._message.data[0] = 1
                self._message.data[1] = 1
                self.pub_auto_cmd.publish(self._message)
                rospy.loginfo('Obstacle left ! vehicle turn right!')
                break
            elif self.range_right < 5 and self.range_center > 5:
                self._message.data[0] = 1
                self._message.data[1] = -1
                self.pub_auto_cmd.publish(self._message)
                rospy.loginfo('Obstacle right ! vehicle turn right!')
                break
            else:
                rospy.loginfo('Hold Tracking!!')
                break

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

    rospy.init_node('autodrive')
    auto_drive = AutoDrive()
    # obst_avoid.test_run()

