#!/usr/bin/env python

import rospy
from learning_service.srv import Person, PersonRequest


def person_client():
    rospy.init_node('person_client')
    rospy.wait_for_service('/show_person')
    try:
        person_client = rospy.ServiceProxy('/show_person', Person)
        response = person_client("Tome", 20, PersonRequest.male)
        return response.result
    except rospy.ServiceProxy as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    print("Show person result: %s" % (person_client()))