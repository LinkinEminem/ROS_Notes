#!/usr/bin/env python

import rospy
from learning_topic.msg import Person


def personInfoCallback(msg):
    rospy.loginfo("Subcribe Person Info: name:%s  age:%d  gender:%d", 
                    msg.name, msg.age, msg.gender)


def person_subscriber():
    rospy.init_node('person_subscriber', anonymous=True)
    rospy.Subscriber("/person_info", Person, personInfoCallback)
    rospy.spin()


if __name__ == '__main__':
    person_subscriber()
