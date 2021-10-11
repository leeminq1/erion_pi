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

DIST_FAR_RANGE = 80
DIST_START_STEER = 60
DIST_STOP_RANGE=20
DIST_BREAK = 0.4

DIST_LAT_ENGAGE = 0.4

K_FRONT_DIST_TO_SPEED = 1.0
K_LAT_DIST_TO_STEER = 2.0

TIME_KEEP_STEERING = 1.5

global f_autodrive

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
        #self.range_center = 6
        #self.range_left = 6
        #self.range_right = 6

        # lidar
        self.range_center = 60
        self.range_left = 60
        self.range_right = 60

        # auto_drive
        self._time_steer = 0
        self._steer_sign_prev = 0

        # camera
        self.obj_arr = []
        self.f_time_person=1
        # time inital
        self._time_detected = 0.0

        # Subscriber
        # ------------------mode info ----------------------#
        self.mode_sub = rospy.Subscriber(
            "roscar_teleop_cmd_vel", Int16MultiArray, self.update_mode)

        #-------------------- sonar -------------------------#
        #self.sub_center = rospy.Subscriber(
        #    "/dkcar/sonar/1", Range, self.update_range)
        #self.sub_left = rospy.Subscriber(
        #    "/dkcar/sonar/0", Range, self.update_range)
        #self.sub_right = rospy.Subscriber(
        #    "/dkcar/sonar/2", Range, self.update_range)
        #rospy.loginfo("Sonar Subscribers set")

        #-------------------- lidar -------------------------#
        self.lidar_sub = rospy.Subscriber(
            "/erion_scan", Int16MultiArray, self.update_lidar_range)

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

    # update time value
    @property
    def is_detected(self):
       # print("time_set")
        return(time.time() - self._time_detected < 1.0)

    def clock(self):
        if (time.time()-self._time_detected > 1.2) or not self.obj_arr:
            self.f_time_person=1
        else:
            self.f_time_person=0
        return self.f_time_person

    # sonar update func
    def update_mode(self, message):
        global f_autodrive
        f_autodrive = message.data[2] == 0
        self.f_time_delay=self.clock()
        if f_autodrive and not self.f_time_delay:
           rospy.loginfo("Auto_drive mode set")
           self.test_run()
        elif self.f_time_delay:
           rospy.loginfo("No Condition")
           self._message.data[0]=0
           self._message.data[1]=0
           self.pub_auto_cmd.publish(self._message)

    # lidar update

    def update_lidar_range(self, message):
        range_arr = message.data
        self.range_left = range_arr[0]
        self.range_center = range_arr[1]
        self.range_right = range_arr[2]

       # rospy.loginfo("left : {}cm , center : {}cm, right : {}cm".format(
       #     self.range_left, self.range_center, self.range_right))

    # camera update func

    def update_object(self, message):
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
       # rospy.loginfo('-----------------------------------------------------------')
       # rospy.loginfo('time : {} , id : {}, score : {:.2f}, box_size_x : {:.2f}, box_size_y :{:.2f} , box_x : {:.2f}, box_y : {:.2f}'.format(self._time_detected,
       #      id, score, bbox_size_x, bbox_size_y, bbox_x, bbox_y))
        # time update
        self._time_detected=time.time()
        # while loop command
       # # we must break to reset f_person value
        #if f_autodrive and self.is_detected:
        #   rospy.loginfo("Working")
        #   self.test_run()
  
    #sonar
    #def update_range(self, message):
    #    angle = message.field_of_view

    #    if abs(angle) > 0:
    #        self.range_center = message.range

    #    elif angle > 0:
    #        self.range_right = message.range

    #    elif angle < 0:
    #        self.range_left = message.range

    #   rospy.loginfo("left : {:.2f}m , center : {:.2f}m".format(
    #       self.range_left, self.range_center))

    def test_run(self):
        rate = rospy.Rate(10)
        # detect condition
        global f_person
        if self.obj_arr[0]==1:
           f_person=1
        else:
           f_person=0
        #f_person = self.obj_arr[0] == 1
        # -- Get the control action
        if f_person and self.is_detected:
	   accel,steer = self.estim_cmd()
           self._message.data[0] = accel
           self._message.data[1] = steer
           self.pub_auto_cmd.publish(self._message)
        elif self.f_time_delay:
           self._message.data[0] = 0
           self._message.data[1] = 0
           self.pub_auto_cmd.publish(self._message)
           rospy.loginfo("Not Detecting")

        rate.sleep()

    def estim_cmd(self):
         # Based on the current ranges / object detecting, calculate the command
	 # init
	 self._message.data[0] = 0
	 self._message.data[1] = 0

	 # --- Get the minimum distance
	 range = min([self.range_center, self.range_left, self.range_right])

	 # --- turn by camera
	 f_camera_left = self.obj_arr[4] < 300
	 f_camera_right = self.obj_arr[4] > 900
	 f_camera_go = 300 < self.obj_arr[4] < 900

	 # --- cmd by laser
	 # stop
	 f_laser_go = range > DIST_FAR_RANGE
	 f_laser_stop = range < DIST_STOP_RANGE
         f_laser_right = (self.range_right < DIST_START_STEER) and (self.range_center > DIST_START_STEER)
         f_laser_right_spin = (self.range_right < DIST_START_STEER) and (self.range_center < DIST_START_STEER)
         f_laser_left = (self.range_left < DIST_START_STEER) and (self.range_center >DIST_START_STEER)
         f_laser_left_spin = (self.range_center < DIST_START_STEER) and (self.range_left < DIST_START_STEER)
	 # stop
	 if f_laser_stop:
	     self._message.data[0] = 0
	     self._message.data[1] = 0
	     rospy.loginfo('-------------Object close ! vehicle stop!!')
	 # turn left
	 elif f_laser_left:
	     self._message.data[0] = 1
	     self._message.data[1] = -1
	     rospy.loginfo('-------------Turn left')
	 # turn left spin
	 elif f_laser_left_spin:
	     self._message.data[0] = 0
	     self._message.data[1] = -1
	     rospy.loginfo('------------Turn left spin')
	 # turn right
	 elif f_laser_right:
	     self._message.data[0] = 1
	     self._message.data[1] = 1
	     rospy.loginfo('-----------Turn right')
	 # turn left spin
	 elif f_laser_right_spin:
	     self._message.data[0] = 0
	     self._message.data[1] = 1
	     rospy.loginfo('-----------Turn right spin')
	 # go
	 elif f_laser_go:
	     self._message.data[0] = 1
	     self._message.data[1] = 0
             rospy.loginfo('----------vehicle go')

	 return (self._message.data[0], self._message.data[1])

    #def get_control_action(self):
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
   # auto_drive.test_run()
