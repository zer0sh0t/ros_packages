#!/usr/bin/env python3
import sys
import rospy
from pkg_zero.srv import AddTwoInts

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        handle = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        res = handle(x, y)
        return res.sum
    except rospy.ServiceException as e:
        rospy.loginfo(f"service call failed: {e}")

if __name__ == '__main__':
    if len(sys.argv) == 3:
        x, y = int(sys.argv[1]), int(sys.argv[2])
    else:
        rospy.logerr("2 arguments are required!!")
        sys.exit(1)

    print(f"requesting {x} + {y}")
    result = add_two_ints_client(x, y)
    print(f"received: {x} + {y} = {result}")
