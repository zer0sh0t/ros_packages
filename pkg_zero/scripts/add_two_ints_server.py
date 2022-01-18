#!/usr/bin/env python3
import rospy
from pkg_zero.srv import AddTwoInts, AddTwoIntsResponse

def add(req):
    res = req.a + req.b
    rospy.loginfo(f"returning {req.a} + {req.b} = {res}")
    return AddTwoIntsResponse(res)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    server = rospy.Service('add_two_ints', AddTwoInts, add)
    rospy.loginfo('ready to add two ints')
    rospy.spin()

if __name__ == '__main__':
    add_two_ints_server()
