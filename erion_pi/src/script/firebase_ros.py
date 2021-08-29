#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sys
print(sys.version)
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore


def set_init():
    cred = credentials.Certificate("erion_key.json")
    firebase_admin.initialize_app(cred)
    db = firestore.client()
    doc_ref = db.collection(u'pi').document(u'key')
    doc = doc_ref.get()
    print("check the initial_value")
    print('Document data: {}'.format(doc.to_dict()))


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)  # 10hz
    firebase_msg = String()
    while not rospy.is_shutdown():
        db = firestore.client()
        doc_ref = db.collection(u'pi').document(u'key')
        doc = doc_ref.get()
        firebase_msg.data = doc.to_dict()['input']
        rospy.loginfo(firebase_msg)
        pub.publish(firebase_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        set_init()
        talker()
    except rospy.ROSInterruptException:
        pass

