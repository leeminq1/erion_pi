#!/usr/bin/env python3
# uploads
# keyboard std_msgs.msg [0]: target_accell_vel  / [1] : target_steering_vel / [2] : operating mode
# operating mode : 0:auto drvie / 1:keyboard control / 2:app control / 3: lift mode

import tty
import termios
import select
from firebase_admin import firestore
from firebase_admin import credentials
import firebase_admin
import rospy
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String

# version chk
import sys
print(sys.version)

# firebase


ROSCAR_MAX_ACCELL_VEL = 255
ROSCAR_MAX_STEERING_VEL = 180

ROSCAR_MIN_ACCELL_VEL = 255
ROSCAR_MIN_STEERING_VEL = 180

msg = """
Control Your ROSCAR!
---------------------------
Moving around:
w
a s d
x

w/x : increase/decrease accell velocity
a/d : increase/decrease steering velocity

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


# firebase func
# get operating mode info
def get_firebase_oper_mode():
    db = firestore.client()
    doc_ref = db.collection(u'pi').document(u'key')
    doc = doc_ref.get()
    oper_mode = doc.to_dict()['oper']
    # print("check the initial_value")
    # print(f'Document data: {oper_mode}')
    return oper_mode


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(target_accell_vel, target_angular_vel):
    return "currently:\taccell vel %s\t steering vel %s " % (target_accell_vel, target_steering_vel)


def constrain(input, low, high):
    if input < low:
        input = low

    elif input > high:
        input = high
    else:
        input = input

    return input


def checkACCELLLimitVelocity(vel):
    vel = constrain(vel, -ROSCAR_MIN_ACCELL_VEL, ROSCAR_MAX_ACCELL_VEL)
    return vel


def checkSTEERINGLimitVelocity(vel):
    vel = constrain(vel, -ROSCAR_MIN_STEERING_VEL, ROSCAR_MAX_STEERING_VEL)
    return vel


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    # firebase init
    cred = credentials.Certificate("erion_key.json")
    firebase_admin.initialize_app(cred)
    print("firebase certificated")

    pub = rospy.Publisher('roscar_teleop_cmd_vel',
                          Int16MultiArray, queue_size=10)
    rospy.init_node('roscar_teleop', anonymous=True)

    teleop_int = Int16MultiArray()
    teleop_int.data = [0, 0, 0]

    status = 0
    target_accell_vel = 0
    target_steering_vel = 0

    try:
        print(msg)
        while (1):
            oper_mode = get_firebase_oper_mode()
            key = getKey()
            if key == 'w':
                target_accell_vel += 1
                target_accell_vel = checkACCELLLimitVelocity(target_accell_vel)
                status = status + 1
                print(vels(target_accell_vel, target_steering_vel))
            elif key == 'x':
                target_accell_vel -= 1
                target_accell_vel = checkACCELLLimitVelocity(target_accell_vel)
                status = status + 1
                print(vels(target_accell_vel, target_steering_vel))
            elif key == 'a':
                target_steering_vel -= 1
                target_steering_vel = checkSTEERINGLimitVelocity(
                    target_steering_vel)
                status = status + 1
                print(vels(target_accell_vel, target_steering_vel))
            elif key == 'd':
                target_steering_vel += 1
                target_steering_vel = checkSTEERINGLimitVelocity(
                    target_steering_vel)
                status = status + 1
                print(vels(target_accell_vel, target_steering_vel))
            elif key == ' ' or key == 's':
                target_accell_vel = 0
                target_steering_vel = 0
                print(vels(target_accell_vel, target_steering_vel))
            elif key == "u":
                oper_mode = 0
            elif key == "i":
                oper_mode = 1
            elif key == "o":
                oper_mode = 2
            elif key == "p":
                oper_mode = 3
            else:
                if (key == '\x03'):
                    break
            teleop_int.data[0] = target_accell_vel
            teleop_int.data[1] = target_steering_vel
            teleop_int.data[2] = oper_mode
            pub.publish(teleop_int)

    except rospy.ROSInterruptException:
        pass

    finally:
        pub.publish(teleop_int)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
